// Native test for lib/ELRSOSD/elrs_osd_core.h.
// g++ -std=c++17 -I lib/ELRSOSD -I lib/ELRSBACKPACK tests/native/elrs_osd_core_test.cpp -o elrs_osd_core_test
// Pass --dump to print frames as hex, for comparing with the VRxC_ELRS plugin.
#include "elrs_osd_core.h"
#include "elrs_backpack_core.h"
#include <cstdio>
#include <cstring>
#include <string>

using namespace ElrsOsdCore;

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)

static std::string hex(const uint8_t* data, size_t n) {
    std::string out;
    char b[4];
    for (size_t i = 0; i < n; ++i) { std::snprintf(b, sizeof(b), "%02x", data[i]); out += b; }
    return out;
}

int main(int argc, char** argv) {
    const bool dump = argc > 1 && std::strcmp(argv[1], "--dump") == 0;
    uint8_t frame[MAX_FRAME];

    // Clear OSD: $X< flag=0 fn=00b6 size=0001 payload=02 crc
    const uint8_t clear[] = {OSD_CLEAR};
    size_t n = buildFrame(SET_OSD, clear, sizeof(clear), frame, sizeof(frame));
    CHECK(n == 10);
    CHECK(frame[0] == '$' && frame[1] == 'X' && frame[2] == '<');
    CHECK(frame[4] == 0xB6 && frame[5] == 0x00 && frame[6] == 1 && frame[7] == 0 && frame[8] == 0x02);
    if (dump) std::printf("clear %s\n", hex(frame, n).c_str());

    const uint8_t text[] = {OSD_WRITE, 14, 19, 0, 'L', 'A', 'P', ' ', '1'};
    n = buildFrame(SET_OSD, text, sizeof(text), frame, sizeof(frame));
    if (dump) std::printf("text %s\n", hex(frame, n).c_str());
    const uint8_t uid[] = {0x01, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
    n = buildFrame(SET_SEND_UID, uid, sizeof(uid), frame, sizeof(frame));
    if (dump) std::printf("uid %s\n", hex(frame, n).c_str());

    // Too big for the backpack's 64-byte buffer is refused.
    uint8_t big[MAX_PAYLOAD + 1] = {};
    CHECK(buildFrame(SET_OSD, big, sizeof(big), frame, sizeof(frame)) == 0);

    // The parser reads back what buildFrame writes, and rejects a bad CRC.
    const uint8_t version[] = {1, 5, 0};
    n = buildFrame(GET_BACKPACK_VERSION, version, sizeof(version), frame, sizeof(frame));
    frame[2] = '>';
    Parser parser;
    bool got = false;
    for (size_t i = 0; i < n; ++i) got = parser.feed(frame[i]);
    CHECK(got && parser.function == GET_BACKPACK_VERSION && parser.size == 3 && parser.payload[1] == 5);
    frame[n - 1] ^= 0xFF;
    got = false;
    for (size_t i = 0; i < n; ++i) got = parser.feed(frame[i]);
    CHECK(!got);

    // The radio's DVR switch as the patched backpack forwards it: the radio's
    // own frame, $X< with function 0x0305 and payload state, delay (u16 LE).
    const uint8_t recording[] = {1, 0, 0};
    n = buildFrame(ElrsBackpackCore::SET_RECORDING_STATE, recording, sizeof(recording), frame, sizeof(frame));
    got = false;
    for (size_t i = 0; i < n; ++i) got = parser.feed(frame[i]);
    CHECK(got && parser.function == 0x0305 && parser.size == 3 && parser.payload[0] == 1);
    CHECK(ElrsBackpackCore::recordingState(frame, n) == 1);

    // Switch on, held past the debounce: one Start; the same position again: nothing.
    ElrsBackpackCore::SwitchControl sw(300);
    sw.received(true, 1000);
    CHECK(sw.poll(1200, false) == ElrsBackpackCore::Command::None);
    CHECK(sw.poll(1300, false) == ElrsBackpackCore::Command::Start);
    sw.received(true, 2000);
    CHECK(sw.poll(2400, false) == ElrsBackpackCore::Command::None);
    sw.received(false, 3000);
    CHECK(sw.poll(3400, false) == ElrsBackpackCore::Command::Stop);

    char t[16];
    formatTime(11534, t, sizeof(t)); CHECK(std::strcmp(t, "11.53") == 0);
    formatTime(62350, t, sizeof(t)); CHECK(std::strcmp(t, "1:02.35") == 0);
    formatTime(999, t, sizeof(t));   CHECK(std::strcmp(t, "1.00") == 0);
    CHECK(centre(10) == 20);

    RaceView race;
    char line[64];
    race.start();
    uint32_t prev = race.add(2310);
    race.lapLine(2310, prev, line, sizeof(line)); CHECK(std::strcmp(line, "GATE 1  2.31") == 0);
    race.bestLine(line, sizeof(line));            CHECK(line[0] == 0);
    prev = race.add(12000);
    race.lapLine(12000, prev, line, sizeof(line)); CHECK(std::strcmp(line, "LAP 1  12.00  BEST") == 0);
    prev = race.add(12500);
    race.lapLine(12500, prev, line, sizeof(line)); CHECK(std::strcmp(line, "LAP 2  12.50  +0.50") == 0);
    prev = race.add(11380);
    race.lapLine(11380, prev, line, sizeof(line)); CHECK(std::strcmp(line, "LAP 3  11.38  BEST") == 0);
    race.bestLine(line, sizeof(line));             CHECK(std::strcmp(line, "BEST 11.38   3 LAPS 35.88") == 0);
    race.add(12000);  // 12.50 + 11.38 + 12.00 = 35.88, not better
    CHECK(race.best3() == 35880);
    race.finishLine(line, sizeof(line));           CHECK(std::strcmp(line, "FINISH  4 LAPS") == 0);

    std::printf(failures ? "%d failure(s)\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}

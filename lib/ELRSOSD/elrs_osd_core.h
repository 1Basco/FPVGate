#pragma once

// Host-testable parts of the goggle OSD link: MSPv2 framing for the ELRS
// timer backpack and the text shown for each race event. No Arduino here.
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

namespace ElrsOsdCore {

// MSP functions understood by the ExpressLRS timer backpack. It forwards
// anything it does not handle itself (SET_OSD) over ESP-NOW to the address
// set by SET_SEND_UID, which is the goggle's binding UID.
constexpr uint16_t GET_BACKPACK_VERSION = 0x0010;
constexpr uint16_t SET_SEND_UID = 0x00B5;
constexpr uint16_t SET_OSD = 0x00B6;

// SET_OSD subcommands, as the RotorHazard VRxC_ELRS plugin sends them to
// HDZero goggles: clear all, write text at (row, col), show what was written.
constexpr uint8_t OSD_CLEAR = 0x02;
constexpr uint8_t OSD_WRITE = 0x03;
constexpr uint8_t OSD_DISPLAY = 0x04;

constexpr uint8_t OSD_COLS = 50;   // HDZero goggle OSD grid
constexpr uint8_t OSD_ROWS = 18;
// The backpack's MSP buffer holds 64 payload bytes; a full row is 4 + 50.
constexpr size_t MAX_PAYLOAD = 4 + OSD_COLS;
constexpr size_t MAX_FRAME = 9 + MAX_PAYLOAD;

inline uint8_t crc(uint8_t value, uint8_t byte) {
    value ^= byte;
    for (unsigned i = 0; i < 8; ++i)
        value = value & 0x80 ? uint8_t((value << 1) ^ 0xD5) : uint8_t(value << 1);
    return value;
}

// One MSPv2 command: $X<, flag, function and size (uint16 LE), payload,
// CRC8 DVB-S2 over flag..payload. Returns the frame length, 0 if too big.
inline size_t buildFrame(uint16_t function, const uint8_t* payload, size_t size, uint8_t* out, size_t cap) {
    if (size > MAX_PAYLOAD || cap < size + 9) return 0;
    out[0] = '$';
    out[1] = 'X';
    out[2] = '<';
    out[3] = 0;
    out[4] = uint8_t(function);
    out[5] = uint8_t(function >> 8);
    out[6] = uint8_t(size);
    out[7] = uint8_t(size >> 8);
    if (size) memcpy(out + 8, payload, size);
    uint8_t checksum = 0;
    for (size_t i = 3; i < size + 8; ++i) checksum = crc(checksum, out[i]);
    out[size + 8] = checksum;
    return size + 9;
}

// Incremental parser for the backpack's replies ($X> frames).
class Parser {
public:
    // Feeds one byte; true when a complete, valid frame has arrived.
    bool feed(uint8_t b) {
        switch (state_) {
            case 0: state_ = b == '$' ? 1 : 0; return false;
            case 1: state_ = b == 'X' ? 2 : 0; return false;
            case 2: state_ = (b == '>' || b == '<' || b == '!') ? 3 : 0; return false;
            case 3: checksum_ = crc(0, b); state_ = 4; return false;
            case 4: function = b; checksum_ = crc(checksum_, b); state_ = 5; return false;
            case 5: function |= uint16_t(b) << 8; checksum_ = crc(checksum_, b); state_ = 6; return false;
            case 6: size = b; checksum_ = crc(checksum_, b); state_ = 7; return false;
            case 7:
                size |= uint16_t(b) << 8;
                checksum_ = crc(checksum_, b);
                length_ = 0;
                if (size > sizeof(payload)) { state_ = 0; return false; }
                state_ = size ? 8 : 9;
                return false;
            case 8:
                payload[length_++] = b;
                checksum_ = crc(checksum_, b);
                if (length_ >= size) state_ = 9;
                return false;
            default: {
                const bool ok = b == checksum_;
                state_ = 0;
                return ok;
            }
        }
    }
    uint16_t function = 0, size = 0;
    uint8_t payload[32] = {};

private:
    uint8_t state_ = 0, checksum_ = 0;
    size_t length_ = 0;
};

// "11.53", or "1:02.35" from a minute up.
inline void formatTime(uint32_t ms, char* out, size_t cap) {
    const uint32_t cs = (ms + 5) / 10;
    const uint32_t minutes = cs / 6000;
    const uint32_t seconds = (cs / 100) % 60;
    if (minutes) snprintf(out, cap, "%lu:%02lu.%02lu", (unsigned long)minutes, (unsigned long)seconds, (unsigned long)(cs % 100));
    else snprintf(out, cap, "%lu.%02lu", (unsigned long)(cs / 100), (unsigned long)(cs % 100));
}

// The column that centres text of this length on the goggle grid.
inline uint8_t centre(size_t length) {
    return length >= OSD_COLS ? 0 : uint8_t((OSD_COLS - length) / 2);
}

// Race state the OSD lines are drawn from. Lap 0 is Gate 1 (start to the
// first pass), excluded from best lap and best three, as in the web UI.
class RaceView {
public:
    static constexpr size_t MAX_LAPS = 128;

    void start() { count_ = 0; best_ = 0; best3_ = 0; }

    // Records one pass. Returns the previous best lap (0 if none), for the
    // delta shown on the lap line.
    uint32_t add(uint32_t lapMs) {
        const uint32_t previousBest = best_;
        if (count_ < MAX_LAPS) laps_[count_] = lapMs;
        ++count_;
        if (count_ > 1) {
            if (!best_ || lapMs < best_) best_ = lapMs;
            const size_t n = count_ < MAX_LAPS ? count_ : MAX_LAPS;
            if (n >= 4) {
                const uint32_t sum = laps_[n - 1] + laps_[n - 2] + laps_[n - 3];
                if (!best3_ || sum < best3_) best3_ = sum;
            }
        }
        return previousBest;
    }

    size_t passes() const { return count_; }
    size_t laps() const { return count_ ? count_ - 1 : 0; }
    uint32_t best() const { return best_; }
    uint32_t best3() const { return best3_; }

    // Top line for the latest pass.
    void lapLine(uint32_t lapMs, uint32_t previousBest, char* out, size_t cap) const {
        char t[16];
        formatTime(lapMs, t, sizeof(t));
        if (count_ <= 1) { snprintf(out, cap, "GATE 1  %s", t); return; }
        if (!previousBest || lapMs < previousBest) { snprintf(out, cap, "LAP %u  %s  BEST", unsigned(laps()), t); return; }
        char d[16];
        formatTime(lapMs - previousBest, d, sizeof(d));
        snprintf(out, cap, "LAP %u  %s  +%s", unsigned(laps()), t, d);
    }

    // Second line: best lap and best three consecutive, once there are any.
    void bestLine(char* out, size_t cap) const {
        out[0] = 0;
        if (!best_) return;
        char b[16];
        formatTime(best_, b, sizeof(b));
        if (!best3_) { snprintf(out, cap, "BEST %s", b); return; }
        char b3[16];
        formatTime(best3_, b3, sizeof(b3));
        snprintf(out, cap, "BEST %s   3 LAPS %s", b, b3);
    }

    void finishLine(char* out, size_t cap) const {
        snprintf(out, cap, "FINISH  %u LAP%s", unsigned(laps()), laps() == 1 ? "" : "S");
    }

private:
    uint32_t laps_[MAX_LAPS] = {};
    size_t count_ = 0;
    uint32_t best_ = 0, best3_ = 0;
};

} // namespace ElrsOsdCore

#include "elrs_osd.h"
#include "config.h"
#include "c5racecontrol.h"
#include "laptimer.h"
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <MD5Builder.h>
#include <Preferences.h>
#include <atomic>

ElrsOsd elrsOsd;

using namespace ElrsOsdCore;

namespace {
constexpr uint32_t BAUD = 460800;          // the timer backpack's serial speed
constexpr uint32_t FRAME_GAP_MS = 3;       // one ESP-NOW send per frame on the backpack
constexpr uint32_t PROBE_MS = 5000;        // version request while idle
constexpr uint32_t SEEN_MS = 12000;        // backpack counts as present this long after a reply
constexpr uint32_t GO_HOLD_MS = 2000;
constexpr uint16_t SET_RECORDING_STATE = ElrsBackpackCore::SET_RECORDING_STATE;
constexpr uint16_t BACKPACK_SET_MODE = 0x0380;   // payload 'W': Wi-Fi update mode

// Version 1 settings, before radio race control.
struct SettingsV1 {
    uint8_t version, enabled, hasUid, row, holdS;
    uint8_t uid[6];
};

// Settings and test requests arrive on the web server's task; the main loop
// applies them, so the queue and the serial port have a single owner.
portMUX_TYPE pendingMux = portMUX_INITIALIZER_UNLOCKED;
std::atomic<bool> settingsPending{false}, testPending{false}, wifiPending{false};
// The last race command, read by the web task.
portMUX_TYPE resultMux = portMUX_INITIALIZER_UNLOCKED;

bool parseByte(AsyncWebServerRequest* req, const char* key, uint8_t& value, uint8_t max) {
    if (!req->hasParam(key, true)) return true;
    const String input = req->getParam(key, true)->value();
    if (input.isEmpty()) return false;
    for (size_t i = 0; i < input.length(); ++i)
        if (!isDigit(input[i])) return false;
    const long parsed = input.toInt();
    if (parsed < 0 || parsed > max) return false;
    value = uint8_t(parsed);
    return true;
}
}

#if defined(PIN_ELRS_OSD_TX) && defined(PIN_ELRS_OSD_RX)
static HardwareSerial& port = Serial2;
bool ElrsOsd::supported() const { return true; }
#else
bool ElrsOsd::supported() const { return false; }
#endif

static ElrsOsd::Settings* pendingSettings() {
    static ElrsOsd::Settings value;
    return &value;
}

void ElrsOsd::begin(LapTimer* timer, Config* config, C5RaceControl* raceControl) {
    timer_ = timer;
    config_ = config;
    raceControl_ = raceControl;
    Preferences prefs;
    if (prefs.begin("elrsosd", true)) {
        const size_t length = prefs.getBytesLength("settings");
        Settings stored;
        stored.version = 0;
        if (length == sizeof(Settings)) {
            if (prefs.getBytes("settings", &stored, sizeof(stored)) != sizeof(stored) ||
                stored.raceControl > 1 || stored.invert > 1) stored.version = 0;
        } else if (length == sizeof(SettingsV1)) {
            // Saved before radio race control: keep the pairing, control off.
            SettingsV1 old;
            if (prefs.getBytes("settings", &old, sizeof(old)) == sizeof(old) && old.version == 1) {
                stored.version = 2;
                stored.enabled = old.enabled;
                stored.hasUid = old.hasUid;
                stored.row = old.row;
                stored.holdS = old.holdS;
                memcpy(stored.uid, old.uid, 6);
            }
        }
        if (stored.version == 2 && stored.enabled <= 1 && stored.hasUid <= 1 &&
            stored.row <= OSD_ROWS - 2 && stored.holdS <= 60) settings_ = stored;
        prefs.end();
    }
    applySettings();
}

void ElrsOsd::applySettings() {
#if defined(PIN_ELRS_OSD_TX) && defined(PIN_ELRS_OSD_RX)
    const bool want = settings_.enabled && settings_.hasUid;
    if (want && !open_) {
        port.setTxBufferSize(1024);
        port.begin(BAUD, SERIAL_8N1, PIN_ELRS_OSD_RX, PIN_ELRS_OSD_TX);
        open_ = true;
        lastProbeMs_ = 0;
    } else if (!want && open_) {
        port.end();
        open_ = false;
    }
    head_ = tail_ = 0;
    clearAt_ = 0;
    if (open_) {
        // The backpack may have restarted on its own; tell it the target
        // again before anything else, then start from a blank OSD.
        sendUid();
        clearAll();
        display();
    }
#endif
}

void ElrsOsd::enqueue(uint16_t function, const uint8_t* payload, size_t size) {
    const size_t next = (tail_ + 1) % QUEUE;
    if (next == head_) { ++framesDropped_; return; }
    Frame& frame = queue_[tail_];
    frame.length = uint8_t(buildFrame(function, payload, size, frame.data, sizeof(frame.data)));
    if (!frame.length) { ++framesDropped_; return; }
    tail_ = next;
}

void ElrsOsd::sendUid() {
    uint8_t payload[7] = {0x01};
    memcpy(payload + 1, settings_.uid, 6);
    enqueue(SET_SEND_UID, payload, sizeof(payload));
}

void ElrsOsd::clearAll() {
    const uint8_t payload[] = {OSD_CLEAR};
    enqueue(SET_OSD, payload, sizeof(payload));
}

void ElrsOsd::clearRow(uint8_t row) {
    uint8_t payload[MAX_PAYLOAD] = {OSD_WRITE, row, 0, 0};
    enqueue(SET_OSD, payload, sizeof(payload));
}

void ElrsOsd::writeRow(uint8_t row, const char* text) {
    clearRow(row);
    const size_t length = strnlen(text, OSD_COLS);
    if (!length) return;
    uint8_t payload[MAX_PAYLOAD] = {OSD_WRITE, row, centre(length), 0};
    memcpy(payload + 4, text, length);
    enqueue(SET_OSD, payload, 4 + length);
}

void ElrsOsd::display() {
    const uint8_t payload[] = {OSD_DISPLAY};
    enqueue(SET_OSD, payload, sizeof(payload));
}

void ElrsOsd::showLines(const char* top, const char* bottom, uint32_t holdMs) {
    sendUid();
    writeRow(settings_.row, top);
    writeRow(settings_.row + 1, bottom);
    display();
    clearAt_ = holdMs ? (millis() + holdMs) | 1 : 0;
}

void ElrsOsd::onLap(uint32_t lapMs) {
    if (!open_) return;
    const uint32_t previousBest = race_.add(lapMs);
    char top[OSD_COLS + 1], bottom[OSD_COLS + 1];
    race_.lapLine(lapMs, previousBest, top, sizeof(top));
    race_.bestLine(bottom, sizeof(bottom));
    showLines(top, bottom, settings_.holdS * 1000u);
}

void ElrsOsd::readReplies(uint32_t now) {
#if defined(PIN_ELRS_OSD_TX) && defined(PIN_ELRS_OSD_RX)
    while (port.available()) {
        if (!parser_.feed(uint8_t(port.read()))) continue;
        if (parser_.function == SET_RECORDING_STATE && parser_.size >= 1) {
            // The radio's DVR Rec switch, forwarded by the backpack.
            lastReplyMs_ = now | 1;
            lastSwitchMs_ = now | 1;
            lastSwitch_ = parser_.payload[0] ? 1 : 0;
            ++switchMessages_;
            switch_.received(lastSwitch_ == 1, now);
            continue;
        }
        if (parser_.function != GET_BACKPACK_VERSION) continue;
        lastReplyMs_ = now | 1;
        const size_t n = parser_.size < sizeof(version_) - 1 ? parser_.size : sizeof(version_) - 1;
        bool text = n > 0;
        for (size_t i = 0; i < n; ++i)
            if (parser_.payload[i] < 0x20 || parser_.payload[i] > 0x7E) text = false;
        if (text) {
            memcpy(version_, parser_.payload, n);
            version_[n] = 0;
        } else {
            size_t used = 0;
            version_[0] = 0;
            for (size_t i = 0; i < n && used < sizeof(version_) - 4; ++i)
                used += snprintf(version_ + used, sizeof(version_) - used, i ? ".%u" : "%u", parser_.payload[i]);
        }
    }
#endif
}

void ElrsOsd::service(uint32_t now) {
    if (settingsPending.exchange(false)) {
        portENTER_CRITICAL(&pendingMux);
        const Settings next = *pendingSettings();
        portEXIT_CRITICAL(&pendingMux);
        const bool reopen = next.enabled != settings_.enabled || next.hasUid != settings_.hasUid;
        settings_ = next;
        if (reopen) applySettings();
        else if (open_) { sendUid(); clearAll(); display(); }
    }
    if (!open_) return;
#if defined(PIN_ELRS_OSD_TX) && defined(PIN_ELRS_OSD_RX)
    readReplies(now);

    if (testPending.exchange(false)) showLines("FPVGATE OSD TEST", "LAP 1  12.34", 5000);
    if (wifiPending.exchange(false)) {
        const uint8_t mode[] = {'W'};
        enqueue(BACKPACK_SET_MODE, mode, sizeof(mode));
    }
    serviceRaceControl(now);

    const bool running = timer_ && timer_->isRaceRunning();
    if (running && !wasRunning_) {
        race_.start();
        sendUid();
        clearAll();
        writeRow(settings_.row, "GO!");
        display();
        clearAt_ = (now + GO_HOLD_MS) | 1;
    } else if (!running && wasRunning_ && race_.passes()) {
        char top[OSD_COLS + 1], bottom[OSD_COLS + 1];
        race_.finishLine(top, sizeof(top));
        race_.bestLine(bottom, sizeof(bottom));
        showLines(top, bottom, settings_.holdS ? settings_.holdS * 2000u : 0);
    } else if (!running && wasRunning_) {
        clearRow(settings_.row);
        clearRow(settings_.row + 1);
        display();
        clearAt_ = 0;
    }
    wasRunning_ = running;

    if (clearAt_ && int32_t(now - clearAt_) >= 0) {
        clearAt_ = 0;
        clearRow(settings_.row);
        clearRow(settings_.row + 1);
        display();
    }

    if (head_ == tail_ && now - lastProbeMs_ >= PROBE_MS) {
        lastProbeMs_ = now;
        enqueue(GET_BACKPACK_VERSION, nullptr, 0);
    }

    // One frame at a time, a few ms apart, never blocking on the UART.
    if (head_ != tail_ && now - lastSendMs_ >= FRAME_GAP_MS) {
        const Frame& frame = queue_[head_];
        if (port.availableForWrite() >= frame.length) {
            port.write(frame.data, frame.length);
            head_ = (head_ + 1) % QUEUE;
            lastSendMs_ = now;
            ++framesSent_;
        }
    }
#endif
}

// The radio's switch to race commands, with ElrsBackpack's rules: debounced,
// one command per new position, C5 races only, start only when idle.
void ElrsOsd::serviceRaceControl(uint32_t now) {
    if (!raceControl_ || !config_) return;
    using ElrsBackpackCore::Command;
    if (startPending_) {
        const C5RaceControl::Status status = raceControl_->status();
        if (!status.busy) {
            startPending_ = false;
            const bool error = status.error && status.error[0];
            result("start", status.running ? "started" : error ? status.error : "startFailed");
        }
    }
    const Command command = switch_.poll(now, settings_.invert);
    if (command == Command::None || !settings_.raceControl) return;
    // Read only when a command is due: it copies the whole roster.
    const C5RaceControl::Status status = raceControl_->status();
    ElrsBackpackCore::RaceState state;
    state.c5 = config_->getReceiverRadio() == 2;
    state.running = status.running;
    state.busy = status.busy;
    state.saveFailed = status.saveFailed;
    const char* reason;
    const Command action = ElrsBackpackCore::decide(command, state, &reason);
    const char* name = command == Command::Start ? "start" : "stop";
    if (action == Command::Start) {
        startPending_ = raceControl_->requestStartAfterCountdown();
        result(name, startPending_ ? "starting" : "notReady");
    } else if (action == Command::Stop) {
        raceControl_->requestStop();
        result(name, "stopping");
    } else {
        result(name, reason);
    }
}

void ElrsOsd::result(const char* command, const char* outcome) {
    portENTER_CRITICAL(&resultMux);
    lastCommand_ = command;
    lastResult_ = outcome;
    lastCommandAt_ = millis() | 1;
    portEXIT_CRITICAL(&resultMux);
}

void ElrsOsd::registerRoutes(AsyncWebServer& server) {
    server.on("/api/elrs-osd", HTTP_GET, [this](AsyncWebServerRequest* req) {
        JsonDocument doc;
        const uint32_t now = millis();
        doc["supported"] = supported();
        doc["enabled"] = bool(settings_.enabled);
        doc["configured"] = bool(settings_.hasUid);
        doc["row"] = settings_.row;
        doc["holdS"] = settings_.holdS;
        doc["open"] = open_;
        doc["backpackSeen"] = lastReplyMs_ && now - lastReplyMs_ < SEEN_MS;
        doc["backpackVersion"] = version_;
        doc["lastReplyAgoMs"] = lastReplyMs_ ? now - lastReplyMs_ : 0;
        doc["framesSent"] = framesSent_;
        doc["framesDropped"] = framesDropped_;
        doc["raceControl"] = bool(settings_.raceControl);
        doc["invert"] = bool(settings_.invert);
        doc["switchMessages"] = switchMessages_;
        doc["lastSwitch"] = lastSwitch_;
        doc["lastSwitchAgoMs"] = lastSwitchMs_ ? now - lastSwitchMs_ : 0;
        portENTER_CRITICAL(&resultMux);
        const char* lastCommand = lastCommand_;
        const char* lastResult = lastResult_;
        const uint32_t lastCommandAt = lastCommandAt_;
        portEXIT_CRITICAL(&resultMux);
        doc["lastCommand"] = lastCommand;
        doc["lastResult"] = lastResult;
        doc["lastCommandAgoMs"] = lastCommandAt ? now - lastCommandAt : 0;
#if defined(PIN_ELRS_OSD_TX) && defined(PIN_ELRS_OSD_RX)
        doc["txPin"] = PIN_ELRS_OSD_TX;
        doc["rxPin"] = PIN_ELRS_OSD_RX;
#endif
        String out;
        serializeJson(doc, out);
        AsyncWebServerResponse* response = req->beginResponse(200, "application/json", out);
        response->addHeader("Cache-Control", "no-store");
        req->send(response);
    });

    server.on("/api/elrs-osd", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (!supported()) {
            req->send(400, "application/json", "{\"error\":\"No backpack UART on this board\"}");
            return;
        }
        Settings candidate = settings_;
        bool valid = parseByte(req, "enabled", candidate.enabled, 1) &&
                     parseByte(req, "row", candidate.row, OSD_ROWS - 2) &&
                     parseByte(req, "holdS", candidate.holdS, 60) &&
                     parseByte(req, "raceControl", candidate.raceControl, 1) &&
                     parseByte(req, "invert", candidate.invert, 1);
        if (valid && req->hasParam("phrase", true)) {
            // ELRS derives the UID from the build flag text, not the bare phrase.
            const String phrase = req->getParam("phrase", true)->value();
            valid = phrase.length() > 0 && phrase.length() <= 63;
            if (valid) {
                MD5Builder md5;
                md5.begin();
                md5.add("-DMY_BINDING_PHRASE=\"" + phrase + "\"");
                md5.calculate();
                uint8_t digest[16];
                md5.getBytes(digest);
                memcpy(candidate.uid, digest, 6);
                candidate.uid[0] &= ~1;  // as the goggle backpack does with its own UID
                candidate.hasUid = 1;
            }
        }
        if (valid && req->hasParam("forget", true) && req->getParam("forget", true)->value() == "1") {
            memset(candidate.uid, 0, 6);
            candidate.hasUid = 0;
        }
        if (!valid || (candidate.enabled && !candidate.hasUid)) {
            req->send(400, "application/json", "{\"error\":\"Invalid settings or missing binding phrase\"}");
            return;
        }
        Preferences prefs;
        bool saved = prefs.begin("elrsosd", false);
        if (saved) {
            saved = prefs.putBytes("settings", &candidate, sizeof(candidate)) == sizeof(candidate);
            prefs.end();
        }
        if (saved) {
            portENTER_CRITICAL(&pendingMux);
            *pendingSettings() = candidate;
            portEXIT_CRITICAL(&pendingMux);
            settingsPending.store(true);
        }
        req->send(saved ? 200 : 500, "application/json",
                  saved ? "{\"ok\":true}" : "{\"error\":\"Cannot save settings\"}");
    });

    server.on("/api/elrs-osd/test", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (!open_) {
            req->send(409, "application/json", "{\"error\":\"Enable the goggle OSD and set the binding phrase first\"}");
            return;
        }
        testPending.store(true);
        req->send(200, "application/json", "{\"ok\":true}");
    });

    // Puts the backpack in its Wi-Fi update mode ("ExpressLRS Timer Backpack",
    // 10.0.0.1), to load new firmware without unsoldering it.
    server.on("/api/elrs-osd/wifi", HTTP_POST, [this](AsyncWebServerRequest* req) {
        if (!open_) {
            req->send(409, "application/json", "{\"error\":\"Enable the goggle OSD and set the binding phrase first\"}");
            return;
        }
        wifiPending.store(true);
        req->send(200, "application/json", "{\"ok\":true}");
    });
}

#pragma once

#include <Arduino.h>
#include "elrs_osd_core.h"
#include "elrs_backpack_core.h"

class AsyncWebServer;
class Config;
class C5RaceControl;
class LapTimer;

// Lap times on HDZero goggles through an ExpressLRS timer backpack (an
// ESP8285/ESP32 flashed with the "Race Timer" backpack firmware) wired to a
// spare UART. The gate sends MSP over serial; the backpack sends it to the
// goggle's backpack over ESP-NOW. Unlike ElrsBackpack, the gate's own Wi-Fi
// is untouched: no channel change, no MAC change, Station mode keeps working.
//
// The same link brings the radio's DVR Rec switch back: the backpack forwards
// MSP_ELRS_BACKPACK_SET_RECORDING_STATE from the radio over serial, and the
// gate starts or stops C5 races with it, with ElrsBackpack's rules. Needs the
// patched ESP8285 timer backpack (stock ESP8285 builds send it back out over
// ESP-NOW instead) built with the radio's binding phrase, and radio and
// goggles on the same phrase: SET_SEND_UID makes the goggle UID the
// backpack's MAC, and the radio sends to its own UID.
class ElrsOsd {
public:
    void begin(LapTimer*, Config*, C5RaceControl*);
    // Main loop: race start/stop edges, clearing after the hold time, the
    // send queue and the backpack's replies.
    void service(uint32_t now);
    // One gate pass, from the main loop's lap broadcast.
    void onLap(uint32_t lapMs);
    void registerRoutes(AsyncWebServer&);
    bool supported() const;

    struct Settings {
        uint8_t version = 2, enabled = 0, hasUid = 0, row = 14, holdS = 5;
        uint8_t uid[6] = {};
        uint8_t raceControl = 0, invert = 0;   // added in version 2
    };

private:
    Settings settings_;

    struct Frame {
        uint8_t length = 0;
        uint8_t data[ElrsOsdCore::MAX_FRAME];
    };
    static constexpr size_t QUEUE = 24;
    Frame queue_[QUEUE];
    size_t head_ = 0, tail_ = 0;
    uint32_t lastSendMs_ = 0;

    LapTimer* timer_ = nullptr;
    Config* config_ = nullptr;
    C5RaceControl* raceControl_ = nullptr;
    ElrsBackpackCore::SwitchControl switch_;
    bool startPending_ = false;
    uint32_t switchMessages_ = 0, lastSwitchMs_ = 0, lastCommandAt_ = 0;
    int8_t lastSwitch_ = -1;
    const char* lastCommand_ = "";
    const char* lastResult_ = "";
    bool open_ = false, wasRunning_ = false;
    uint32_t clearAt_ = 0, lastProbeMs_ = 0, lastReplyMs_ = 0;
    uint32_t framesSent_ = 0, framesDropped_ = 0;
    char version_[16] = "";
    ElrsOsdCore::Parser parser_;
    ElrsOsdCore::RaceView race_;

    void applySettings();
    void enqueue(uint16_t function, const uint8_t* payload, size_t size);
    void sendUid();
    void clearAll();
    void writeRow(uint8_t row, const char* text);
    void clearRow(uint8_t row);
    void display();
    void showLines(const char* top, const char* bottom, uint32_t holdMs);
    void readReplies(uint32_t now);
    void serviceRaceControl(uint32_t now);
    void result(const char* command, const char* outcome);
};

extern ElrsOsd elrsOsd;

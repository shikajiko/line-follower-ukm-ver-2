#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

// NOTE: AP_SSID / AP_PASSWORD are no longer used directly for the actual
// hotspot (that now comes from wifi_config.h, is user-editable, and gets a
// unique per-device suffix). They're kept here only as compile-time
// fallbacks in case other code in the project still references them.
#ifndef AP_SSID
#define AP_SSID "LineBot-Setup"
#endif
#ifndef AP_PASSWORD
#define AP_PASSWORD "line12345678"
#endif

// ---- Filesystem / mission persistence ----
bool mountFilesystem();
bool loadMissionFile();
bool parseMissionJSON(const JsonDocument &doc);
extern bool justLoadedMission;

// ---- HTTP server lifecycle ----
void startMissionWebServer();
void handleMissionWebServer();

// ---- WiFi lifecycle ----
// Call this once from setup() instead of the old enableHotspot(). It brings
// up the robot's own hotspot, attempts to join a saved home/phone network
// as a station (if one is configured), starts mDNS when that succeeds, and
// finally starts the mission web server. Safe to call more than once.
void enableHotspot();

// ---- Status ----
bool isMissionLoaded();
bool isWebServerStarted();

void printHotspotInformation();
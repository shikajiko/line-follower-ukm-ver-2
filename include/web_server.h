#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdint.h>
#include <stddef.h>
#include "mission.h"
#include <ArduinoJson.h>

#define AP_SSID "LF_1"
#define AP_PASSWORD "icH12o2026"

#define MISSION_RECORD_SIZE 15

// HTTP route handlers
void handleIndex();
void handleUpdateMission();
void handleLoadMission();
void handleNotFound();

// Lifecycle
void enableHotspot();
void startMissionWebServer();
void handleMissionWebServer();

bool loadMissionFile();
bool isWebServerStarted();
bool isMissionLoaded();
bool parseMissionJSON(const JsonDocument &doc);
bool mountFilesystem();

#endif
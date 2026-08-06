#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdint.h>
#include <stddef.h>
#include "mission.h"

#define AP_SSID "LINE_FOLLOWER_1"
#define AP_PASSWORD "icH12o2026"

#define MISSION_RECORD_SIZE 19

// HTTP route handlers
void handleIndex();
void handleLoadMission();
void handleSaveMission();
void handleNotFound();

// Lifecycle
void enableHotspot();          
void startMissionWebServer();  
void handleMissionWebServer(); 
void loadMissionsFromNVS();

bool isWebserverStarted();
bool isMissionLoaded();

#endif
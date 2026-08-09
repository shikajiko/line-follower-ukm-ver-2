#include "web_server.h"
#include "mission.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <string.h>
#include "display.h"
#include <ArduinoJson.h>

static WebServer webServer(80);
static bool web_server_started = false;
static bool is_mission_loaded = false;
static bool fs_mounted = false;

#define MISSION_PREF_NAMESPACE "missions"
#define MISSION_PREF_KEY "data"
#define MISSION_PREF_COUNT_KEY "count"

#define INDEX_HTML_PATH "/index.html"
#define MISSION_JSON_PATH "/missions.json"

bool loadMissionFile()
{
    if (!mountFilesystem())
    {
        Serial.println("[WEB] Failed to mount LittleFS");
        return false;
    }

    if (!LittleFS.exists(MISSION_JSON_PATH))
    {
        Serial.println("[WEB] missions.json not found");
        return false;
    }

    File file = LittleFS.open(MISSION_JSON_PATH, "r");
    if (!file)
    {
        Serial.println("[WEB] Failed to open missions.json");
        return false;
    }

    JsonDocument doc;

    DeserializationError err = deserializeJson(doc, file);
    file.close();

    if (err)
    {
        Serial.print("[WEB] Failed to parse missions.json: ");
        Serial.println(err.c_str());
        return false;
    }

    if (!parseMissionJSON(doc))
    {
        Serial.println("[WEB] Invalid mission data");
        return false;
    }

    is_mission_loaded = true;

    Serial.printf("[WEB] Loaded %d missions from LittleFS\n", NUM_STATES);

    return true;
}

bool parseMissionJSON(const JsonDocument &doc)
{
    JsonArrayConst missions;
    int start = (doc["start_from"] | 1) - 1;

    if (doc.is<JsonArrayConst>())
    {
        missions = doc.as<JsonArrayConst>();
    }
    else
    {
        missions = doc["missions"];
    }

    if (missions.isNull())
    {
        Serial.println("No missions array found.");
        return false;
    }

    NUM_STATES = 0;

    for (JsonObjectConst m : missions)
    {
        if (NUM_STATES >= MAX_MISSIONS)
            break;

        MissionState &state = missionStates[NUM_STATES];

        //-------------------------
        // Line mode
        //-------------------------
        const char *lineMode = m["lineMode"] | "LINE_BLACK";

        if (!strcmp(lineMode, "LINE_WHITE"))
            state.lineMode = LINE_WHITE;
        else
            state.lineMode = LINE_BLACK;

        //-------------------------
        // Drive mode
        //-------------------------
        const char *driveMode = m["driveMode"] | "DIRECT_MOVE";

        if (!strcmp(driveMode, "DIRECT_MOVE"))
            state.driveMode = DIRECT_MOVE;
        else if (!strcmp(driveMode, "PID_STRAIGHT"))
            state.driveMode = PID_STRAIGHT;
        else if (!strcmp(driveMode, "PID"))
            state.driveMode = PID;
        else
            state.driveMode = DIRECT_MOVE;

        //-------------------------
        // Speeds
        //-------------------------
        state.leftSpeed = m["leftSpeed"] | 0;
        state.rightSpeed = m["rightSpeed"] | 0;

        //-------------------------
        // Condition
        //-------------------------
        const char *condition = m["condition"] | "COND_IMMEDIATE";

        if (!strcmp(condition, "COND_ENCODER1_GT"))
            state.condition = COND_ENCODER1_GT;
        else if (!strcmp(condition, "COND_ENCODER2_GT"))
            state.condition = COND_ENCODER2_GT;
        else if (!strcmp(condition, "COND_DIST_GT"))
            state.condition = COND_DIST_GT;
        else if (!strcmp(condition, "COND_SENSOR_MASK"))
            state.condition = COND_SENSOR_MASK;
        else if (!strcmp(condition, "COND_TIMER"))
            state.condition = COND_TIMER;
        else
            state.condition = COND_IMMEDIATE;

        //-------------------------
        // Threshold
        //-------------------------
        state.condition_threshold = m["condition_threshold"] | 0;

        //-------------------------
        // Sensor masks
        //-------------------------
        state.sensorLeft = m["sensorLeft"] | 0;
        state.sensorRight = m["sensorRight"] | 0;

        //-------------------------
        // Mask mode
        //-------------------------
        const char *maskMode = m["maskMode"] | "MASK_AND";

        if (!strcmp(maskMode, "MASK_OR"))
            state.maskMode = MASK_OR;
        else
            state.maskMode = MASK_AND;

        //-------------------------
        // Stop mode
        //-------------------------
        const char *stopMode = m["stopMode"] | "NONE";

        if (!strcmp(stopMode, "STOP"))
            state.stopMode = STOP;
        else if (!strcmp(stopMode, "BRAKE"))
            state.stopMode = BRAKE;
        else
            state.stopMode = NONE;

        NUM_STATES++;

        state.is_checkpoint = m["is_checkpoint"];
    }

    setCurrentMission(start);
    updateStartIndex(start);
    return true;
}

bool mountFilesystem() {
    if (fs_mounted) return true;
    if (!LittleFS.begin(true)) {
        Serial.println("[WEB] LittleFS mount failed");
        return false;
    }
    fs_mounted = true;
    Serial.println("[WEB] LittleFS mounted");
    return true;
}

void handleIndex() {
    if (!fs_mounted || !LittleFS.exists(INDEX_HTML_PATH)) {
        webServer.send(500, "text/plain",
            "index.html not found on LittleFS. Upload the filesystem image "
            "(data/index.html) to the device first.");
        return;
    }

    File f = LittleFS.open(INDEX_HTML_PATH, "r");
    if (!f) {
        webServer.send(500, "text/plain", "Failed to open index.html");
        return;
    }

    webServer.sendHeader("Cache-Control", "no-store");
    webServer.streamFile(f, "text/html");
    f.close();
}

void handleLoadMission() {
    File f = LittleFS.open(MISSION_JSON_PATH, "r");
    if (!f) {
        webServer.send(404, "text/plain", "Not found");
        return;
    }

    webServer.streamFile(f, "application/json");
    f.close();
}
void handleUpdateMission()
{
    if (!webServer.hasArg("plain"))
    {
        webServer.send(400, "text/plain", "Missing request body");
        return;
    }

    String json = webServer.arg("plain");

    JsonDocument doc;

    DeserializationError err = deserializeJson(doc, json);

    if (err)
    {
        Serial.print("JSON parse failed: ");
        Serial.println(err.c_str());

        webServer.send(400, "text/plain", "Invalid JSON");
        return;
    }

    if (!parseMissionJSON(doc))
    {
        webServer.send(400, "text/plain", "Invalid mission data");
        return;
    }

    File file = LittleFS.open(MISSION_JSON_PATH, "w");

    if (!file)
    {
        webServer.send(500, "text/plain", "Failed to open mission file");
        return;
    }

    serializeJson(doc, file);
    file.close();

    is_mission_loaded = true;

    displayOLED("MISSION", "LOADED", "", "");

    Serial.printf("Loaded %d missions.\n", NUM_STATES);

    webServer.send(200, "text/plain", "Mission saved");
}

void handleNotFound() {
    webServer.send(404, "text/plain", "Not found");
}

void startMissionWebServer() {
    if (web_server_started) return;

    mountFilesystem();

    static const char *headerKeys[] = { "Content-Length" };
    webServer.collectHeaders(headerKeys, 1);

    webServer.on("/", HTTP_GET, handleIndex);
    webServer.on("/load", HTTP_GET, handleLoadMission);
    webServer.on("/save", HTTP_POST, handleUpdateMission);
    webServer.onNotFound(handleNotFound);

    webServer.begin();
    web_server_started = true;
    Serial.println("[WEB] Mission editor server started on port 80");
}

void handleMissionWebServer() {
    if (web_server_started) {
        webServer.handleClient();
    }
}

static IPAddress ip;

void enableHotspot() {
    if (!web_server_started) {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);

        ip = WiFi.softAPIP();
    }

    char line1buf[24];
    char line2buf[24];
    char line3buf[24];

    snprintf(line1buf, sizeof(line1buf), "Hotspot \"%s\"", AP_SSID);
    snprintf(line2buf, sizeof(line2buf), "\"%s\"", AP_PASSWORD);
    snprintf(line3buf, sizeof(line3buf), "http://%u.%u.%u.%u/\n", ip[0], ip[1], ip[2], ip[3]);

    displayOLED(line1buf, line2buf, line3buf, "");

    Serial.printf("[WEB] Hotspot \"%s\" up connect and browse to http://%u.%u.%u.%u/\n",
                  AP_SSID, ip[0], ip[1], ip[2], ip[3]);

    startMissionWebServer();
}

bool isMissionLoaded() {
    return is_mission_loaded;
}

bool isWebServerStarted() {
    return web_server_started;
}
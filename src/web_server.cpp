#include "web_server.h"
#include "wifi_config.h"
#include "mission.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <string.h>
#include "display.h"
#include <ArduinoJson.h>

static WebServer webServer(80);
static bool web_server_started = false;
static bool is_mission_loaded = false;
bool justLoadedMission = false;
static bool fs_mounted = false;
static bool sta_connected = false;

static String apSsid;
static String apPassword;
static String hostname;

static IPAddress apIp;
static IPAddress staIp;
   
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

    displayOLED("MISSION", "LOADED", "PRESS BTN 3", "TO MENU");
    justLoadedMission = true;

    Serial.printf("Loaded %d missions.\n", NUM_STATES);

    webServer.send(200, "text/plain", "Mission saved");
}

// ---------------------------------------------------------------------
// WiFi configuration endpoints
//
// GET  /wifi  -> current status/settings (never returns saved passwords)
// POST /wifi  -> update sta_ssid/sta_password and/or ap_ssid_prefix/
//                ap_password. Any field left out of the JSON body is left
//                unchanged. On success the device reboots to apply the
//                new settings.
// ---------------------------------------------------------------------

void handleGetWifiConfig()
{
    JsonDocument doc;

    String staSsid = wifiGetStaSsid();
    doc["sta_ssid"] = staSsid;
    doc["sta_connected"] = sta_connected;
    doc["sta_ip"] = sta_connected ? WiFi.localIP().toString() : "";

    doc["ap_ssid"] = wifiGetApSsid();
    doc["ap_ssid_prefix"] = wifiGetApSsidPrefix();
    doc["ap_ip"] = WiFi.softAPIP().toString();

    doc["hostname"] = wifiGetHostname();
    doc["mac_suffix"] = wifiGetMacSuffix();

    String out;
    serializeJson(doc, out);
    webServer.send(200, "application/json", out);
}

void handleSetWifiConfig()
{
    if (!webServer.hasArg("plain"))
    {
        webServer.send(400, "text/plain", "Missing request body");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, webServer.arg("plain"));
    if (err)
    {
        webServer.send(400, "text/plain", "Invalid JSON");
        return;
    }

    bool changed = false;

    // Station (home/phone WiFi) credentials.
    if (!doc["sta_ssid"].isNull())
    {
        String ssid = doc["sta_ssid"] | "";
        String pass = doc["sta_password"] | "";

        if (ssid.length() == 0)
        {
            wifiClearStaCredentials();
        }
        else if (!wifiSaveStaCredentials(ssid, pass))
        {
            webServer.send(400, "text/plain",
                "Invalid WiFi name/password (password must be empty or at least 8 characters)");
            return;
        }
        changed = true;
    }

    // Robot's own hotspot settings.
    if (!doc["ap_ssid_prefix"].isNull())
    {
        String prefix = doc["ap_ssid_prefix"] | "";
        String pass = doc["ap_password"] | "";

        if (!wifiSaveApSettings(prefix, pass))
        {
            webServer.send(400, "text/plain",
                "Invalid hotspot name/password (name required, password must be empty or at least 8 characters)");
            return;
        }
        changed = true;
    }

    if (!changed)
    {
        webServer.send(400, "text/plain", "Nothing to update");
        return;
    }

    webServer.send(200, "text/plain", "Saved. Rebooting to apply new WiFi settings.");
    displayOLED("WIFI CONFIG", "SAVED", "REBOOTING...", "");

    delay(300); // let the response flush before we drop the connection
    ESP.restart();
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
    webServer.on("/wifi", HTTP_GET, handleGetWifiConfig);
    webServer.on("/wifi", HTTP_POST, handleSetWifiConfig);
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

static bool connectStationMode()
{
    String ssid = wifiGetStaSsid();
    String pass = wifiGetStaPassword();

    if (ssid.length() == 0)
        return false;

    Serial.printf("[WEB] Attempting to join \"%s\" as station...\n", ssid.c_str());

    if (pass.length() == 0)
        WiFi.begin(ssid.c_str());
    else
        WiFi.begin(ssid.c_str(), pass.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_STA_CONNECT_TIMEOUT_MS)
    {
        delay(250);
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("[WEB] Station connected, IP = ");
        Serial.println(WiFi.localIP());
        return true;
    }

    Serial.println("[WEB] Station connect timed out, staying on hotspot only");
    return false;
}

void enableHotspot() {
    if (web_server_started) {
        return;
    }

    wifiConfigInit();
    WiFi.mode(WIFI_AP_STA);

    apSsid = wifiGetApSsid();
    apPassword = wifiGetApPassword();

    if (apPassword.length() == 0)
        WiFi.softAP(apSsid.c_str());
    else
        WiFi.softAP(apSsid.c_str(), apPassword.c_str());

    apIp = WiFi.softAPIP();

    sta_connected = connectStationMode();
    hostname = wifiGetHostname();

    startMissionWebServer();
}

bool isMissionLoaded() {
    return is_mission_loaded;
}

bool isWebServerStarted() {
    return web_server_started;
}

void printHotspotInformation() {    
    char line1buf[24];
    char line2buf[24];
    char line3buf[24];
    char line4buf[24];

    if (sta_connected)
    {
        staIp = WiFi.localIP();
        snprintf(line1buf, sizeof(line1buf), "WiFi \"%s\"", wifiGetStaSsid().c_str());
        snprintf(line2buf, sizeof(line2buf), "http://%u.%u.%u.%u/", staIp[0], staIp[1], staIp[2], staIp[3]);

        Serial.printf("[WEB] Also reachable via hotspot \"%s\" at http://%u.%u.%u.%u/\n",
                      apSsid.c_str(), apIp[0], apIp[1], apIp[2], apIp[3]);
    }
    else
    {
        snprintf(line1buf, sizeof(line1buf), "Hotspot \"%s\"", apSsid.c_str());
        snprintf(line2buf, sizeof(line2buf), "%s", apPassword.length() ? apPassword.c_str() : "(open network)");
        snprintf(line3buf, sizeof(line3buf), "http://%u.%u.%u.%u/", apIp[0], apIp[1], apIp[2], apIp[3]);
        line4buf[0] = '\0';
    }

    displayOLED(line1buf, line2buf, line3buf, line4buf);

    Serial.printf("[WEB] Hotspot \"%s\" up, connect and browse to http://%u.%u.%u.%u/\n",
                  apSsid.c_str(), apIp[0], apIp[1], apIp[2], apIp[3]);
}
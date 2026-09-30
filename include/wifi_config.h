#pragma once

#include <Arduino.h>

// Fallback defaults used only the very first time the robot boots, before
// the user has saved anything through the WiFi Setup panel.
#define WIFI_DEFAULT_AP_SSID_PREFIX "LineBot"
#define WIFI_DEFAULT_AP_PASSWORD "line12345678" // >= 8 chars required for WPA2

// How long to wait for a station-mode connection before giving up and
// falling back to AP-only.
#define WIFI_STA_CONNECT_TIMEOUT_MS 4000

// Loads saved settings from NVS (Preferences). Safe to call multiple times;
// only does real work once.
void wifiConfigInit();

// A short, stable, unique suffix derived from the chip's MAC address, e.g.
// "3F2A1C". Used to guarantee no two robots share an SSID / mDNS hostname
// even if they have the same configured name.
String wifiGetMacSuffix();

// The AP SSID actually broadcast: "<prefix>-<macSuffix>".
String wifiGetApSsid();

// Just the user-editable part of the AP SSID (no MAC suffix).
String wifiGetApSsidPrefix();

String wifiGetApPassword();

// Station (home/phone WiFi) credentials the robot will try to join.
String wifiGetStaSsid();
String wifiGetStaPassword();
bool wifiHasStaCredentials();

// mDNS hostname, e.g. "linebot-3f2a1c" (robot reachable at
// "linebot-3f2a1c.local"). Lowercase, alnum + hyphen only.
String wifiGetHostname();

// Persist new settings. Each returns false if validation fails (bad
// password length, empty required field, etc). Callers are expected to
// reboot after a successful save so the new settings take effect.
bool wifiSaveStaCredentials(const String &ssid, const String &password);
bool wifiClearStaCredentials();
bool wifiSaveApSettings(const String &ssidPrefix, const String &password);
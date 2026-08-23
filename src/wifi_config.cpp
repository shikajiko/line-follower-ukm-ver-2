#include "wifi_config.h"

#include <Preferences.h>
#include <WiFi.h>

#define WIFI_PREF_NAMESPACE "wifi_cfg"
#define KEY_STA_SSID "sta_ssid"
#define KEY_STA_PASS "sta_pass"
#define KEY_AP_PREFIX "ap_prefix"
#define KEY_AP_PASS "ap_pass"

static bool s_initialized = false;
static String s_macSuffix;

static String toLowerAlnumHyphen(const String &in)
{
    String out;
    out.reserve(in.length());
    for (size_t i = 0; i < in.length(); i++)
    {
        char c = in[i];
        if (isalnum((unsigned char)c))
        {
            out += (char)tolower((unsigned char)c);
        }
        else if (c == '-' || c == '_' || c == ' ')
        {
            out += '-';
        }
        // any other character is dropped
    }
    if (out.length() == 0)
        out = "robot";
    return out;
}

void wifiConfigInit()
{
    if (s_initialized)
        return;

    // Derive the MAC-based suffix once, from the STA MAC (stable regardless
    // of which mode is active).
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char buf[7];
    snprintf(buf, sizeof(buf), "%02X%02X%02X", mac[3], mac[4], mac[5]);
    s_macSuffix = String(buf);

    s_initialized = true;
    Serial.printf("[WIFI_CFG] Initialized, MAC suffix = %s\n", s_macSuffix.c_str());
}

String wifiGetMacSuffix()
{
    wifiConfigInit();
    return s_macSuffix;
}

String wifiGetApSsidPrefix()
{
    Preferences prefs;
    prefs.begin(WIFI_PREF_NAMESPACE, true);
    String prefix = prefs.getString(KEY_AP_PREFIX, WIFI_DEFAULT_AP_SSID_PREFIX);
    prefs.end();
    if (prefix.length() == 0)
        prefix = WIFI_DEFAULT_AP_SSID_PREFIX;
    return prefix;
}

String wifiGetApSsid()
{
    wifiConfigInit();
    return wifiGetApSsidPrefix() + "-" + s_macSuffix;
}

String wifiGetApPassword()
{
    Preferences prefs;
    prefs.begin(WIFI_PREF_NAMESPACE, true);
    // getString with a default that's never returned when the key was
    // explicitly saved as empty (open network), so we check exists() first.
    String pass;
    if (prefs.isKey(KEY_AP_PASS))
        pass = prefs.getString(KEY_AP_PASS, "");
    else
        pass = WIFI_DEFAULT_AP_PASSWORD;
    prefs.end();
    return pass;
}

String wifiGetStaSsid()
{
    Preferences prefs;
    prefs.begin(WIFI_PREF_NAMESPACE, true);
    String ssid = prefs.getString(KEY_STA_SSID, "");
    prefs.end();
    return ssid;
}

String wifiGetStaPassword()
{
    Preferences prefs;
    prefs.begin(WIFI_PREF_NAMESPACE, true);
    String pass = prefs.getString(KEY_STA_PASS, "");
    prefs.end();
    return pass;
}

bool wifiHasStaCredentials()
{
    return wifiGetStaSsid().length() > 0;
}

String wifiGetHostname()
{
    wifiConfigInit();
    String prefix = toLowerAlnumHyphen(wifiGetApSsidPrefix());
    return prefix + "-" + s_macSuffix;
}

bool wifiSaveStaCredentials(const String &ssid, const String &password)
{
    if (ssid.length() == 0 || ssid.length() > 32)
        return false;
    if (password.length() > 0 && password.length() < 8)
        return false;
    if (password.length() > 64)
        return false;

    Preferences prefs;
    prefs.begin(WIFI_PREF_NAMESPACE, false);
    prefs.putString(KEY_STA_SSID, ssid);
    prefs.putString(KEY_STA_PASS, password);
    prefs.end();

    Serial.printf("[WIFI_CFG] Saved STA credentials for \"%s\"\n", ssid.c_str());
    return true;
}

bool wifiClearStaCredentials()
{
    Preferences prefs;
    prefs.begin(WIFI_PREF_NAMESPACE, false);
    prefs.remove(KEY_STA_SSID);
    prefs.remove(KEY_STA_PASS);
    prefs.end();

    Serial.println("[WIFI_CFG] Cleared saved STA credentials");
    return true;
}

bool wifiSaveApSettings(const String &ssidPrefix, const String &password)
{
    if (ssidPrefix.length() == 0 || ssidPrefix.length() > 20)
        return false;
    if (password.length() > 0 && password.length() < 8)
        return false;
    if (password.length() > 64)
        return false;

    Preferences prefs;
    prefs.begin(WIFI_PREF_NAMESPACE, false);
    prefs.putString(KEY_AP_PREFIX, ssidPrefix);
    prefs.putString(KEY_AP_PASS, password);
    prefs.end();

    Serial.printf("[WIFI_CFG] Saved AP settings, prefix = \"%s\"\n", ssidPrefix.c_str());
    return true;
}
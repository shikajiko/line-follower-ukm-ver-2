#include "display.h"
#include "stdint.h"
#include "string.h"
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

void displayOLED(const char *line1, const char *line2, const char *line3,
                 const char *line4) {
  static char prev1[64] = "";
  static char prev2[64] = "";
  static char prev3[64] = "";
  static char prev4[64] = "";
  static uint32_t last_serial_ms = 0;
  static bool first_frame = true;

  const char *l1 = line1 ? line1 : "";
  const char *l2 = line2 ? line2 : "";
  const char *l3 = line3 ? line3 : "";
  const char *l4 = line4 ? line4 : "";

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (line1)
    display.println(line1);
  if (line2)
    display.println(line2);
  if (line3)
    display.println(line3);
  if (line4)
    display.println(line4);

  display.display();

  bool changed = (strcmp(prev1, l1) != 0) || (strcmp(prev2, l2) != 0) ||
                 (strcmp(prev3, l3) != 0) || (strcmp(prev4, l4) != 0);

  uint32_t now = millis();
  bool time_ok = (now - last_serial_ms) >= 200;

  if (first_frame || (changed && time_ok)) {
    Serial.println("[OLED]");
    Serial.println(l1);
    Serial.println(l2);
    Serial.println(l3);
    Serial.println(l4);
    Serial.println("---");

    snprintf(prev1, sizeof(prev1), "%s", l1);
    snprintf(prev2, sizeof(prev2), "%s", l2);
    snprintf(prev3, sizeof(prev3), "%s", l3);
    snprintf(prev4, sizeof(prev4), "%s", l4);
    last_serial_ms = now;
    first_frame = false;
  }
}
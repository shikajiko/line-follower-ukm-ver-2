#include "display.h"
#include "stdint.h"
#include "string.h"
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------------------------------------------------------------------
// Plain 4-line status screen (unchanged) — used by every non-menu state:
// calibration, line debug, PID tuning, countdowns, errors, etc.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Highlighted scrolling menu — title bar, up to MENU_VISIBLE_ROWS items with
// the selection drawn as an inverted (filled) row, scroll-position carets on
// the right edge when the list overflows, and a footer showing the button
// legend. Used only for the main mode-select menu.
// ---------------------------------------------------------------------------
#define MENU_VISIBLE_ROWS 3
#define MENU_ROW_HEIGHT 12
#define MENU_LIST_TOP 13
#define MENU_ARROW_W 5

void displayMenuOLED(const char *title, const char *const *items,
                      uint8_t item_count, uint8_t selected_index) {
  display.clearDisplay();
  display.setTextSize(1);

  // --- Title bar -----------------------------------------------------
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(2, 1);
  display.print(title);
  display.drawFastHLine(0, 11, SCREEN_WIDTH, SSD1306_WHITE);

  // --- Scroll window: keep the selection visible ----------------------
  uint8_t visible =
      (item_count < MENU_VISIBLE_ROWS) ? item_count : MENU_VISIBLE_ROWS;

  int16_t window_start = (int16_t)selected_index - visible / 2;
  int16_t max_start = (int16_t)item_count - (int16_t)visible;
  if (window_start > max_start)
    window_start = max_start;
  if (window_start < 0)
    window_start = 0;

  // --- Rows ------------------------------------------------------------
  for (uint8_t row = 0; row < visible; row++) {
    uint8_t idx = (uint8_t)(window_start + row);
    int16_t y = MENU_LIST_TOP + row * MENU_ROW_HEIGHT;
    bool selected = (idx == selected_index);

    if (selected) {
      display.fillRoundRect(0, y, SCREEN_WIDTH - MENU_ARROW_W - 2,
                             MENU_ROW_HEIGHT - 2, 2, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(6, y + 2);
    display.print(items[idx]);
  }

  // --- Scroll carets (only shown when content is hidden above/below) --
  display.setTextColor(SSD1306_WHITE);
  int16_t arrow_x = SCREEN_WIDTH - MENU_ARROW_W;
  if (window_start > 0) {
    display.fillTriangle(arrow_x, MENU_LIST_TOP + 4, arrow_x + MENU_ARROW_W,
                          MENU_LIST_TOP + 4, arrow_x + (MENU_ARROW_W / 2),
                          MENU_LIST_TOP - 1, SSD1306_WHITE);
  }
  if (window_start + visible < item_count) {
    int16_t base_y = MENU_LIST_TOP + visible * MENU_ROW_HEIGHT;
    display.fillTriangle(arrow_x, base_y - 5, arrow_x + MENU_ARROW_W,
                          base_y - 5, arrow_x + (MENU_ARROW_W / 2), base_y,
                          SSD1306_WHITE);
  }

  // --- Footer: button legend -------------------------------------------
  int16_t footer_y = SCREEN_HEIGHT - 8;
  display.drawFastHLine(0, footer_y - 2, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(2, footer_y);
  display.print("2:UP 1:DN 4:OK");

  display.display();

  // --- Serial mirror (throttled, change-detected like displayOLED) ----
  static uint8_t prev_selected = 0xFF;
  static uint32_t last_serial_ms = 0;
  uint32_t now = millis();

  if (prev_selected != selected_index && (now - last_serial_ms) >= 150) {
    Serial.println("[OLED MENU]");
    Serial.println(title);
    for (uint8_t row = 0; row < visible; row++) {
      uint8_t idx = (uint8_t)(window_start + row);
      Serial.print(idx == selected_index ? "> " : "  ");
      Serial.println(items[idx]);
    }
    Serial.println("---");
    prev_selected = selected_index;
    last_serial_ms = now;
  }
}
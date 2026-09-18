#ifndef DISPLAY_H
#define DISPLAY_H

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#include <stdint.h>

void displayOLED(const char *line1, const char *line2, const char *line3,
                 const char *line4);

void displayMenuOLED(const char *title, const char *const *items,
                      uint8_t item_count, uint8_t selected_index);

#endif //display.h
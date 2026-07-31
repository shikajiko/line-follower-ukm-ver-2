#ifndef DISPLAY_H
#define DISPLAY_H

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

void displayOLED(const char *line1, const char *line2, const char *line3,
                 const char *line4);

#endif //display.h
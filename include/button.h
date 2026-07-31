#ifndef BUTTON_H
#define BUTTON_H

#include "IO.h"
#include "stdint.h"

enum ButtonId {
  BTN_1 = 0,
  BTN_2 = 1,
  BTN_3 = 2,
  BTN_4 = 3,
};

struct Button {
    uint8_t pin;
    int stableState;
    int rawState;
    uint32_t changeTime;

    bool pressedEvent;
    bool releasedEvent;

    uint32_t holdStart;
    bool holdLatched;
};

static Button buttons[4];

void initButtonStruct();
void pollButtons();

bool buttonHeld(uint8_t id, uint32_t ms);
bool isButtonPressed(uint8_t id);
bool isButtonReleased(uint8_t id);
bool isButtonDown(uint8_t id);
bool isButtonUp(uint8_t id);
uint32_t buttonHeldMs(uint8_t id);

#endif // button.h
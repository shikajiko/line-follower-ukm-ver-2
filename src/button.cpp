#include "button.h"
#include <Arduino.h>

uint32_t button_debounce_time = 50;

void initButtonStruct() {
    buttons[0].pin = BUTTON1_PIN;
    buttons[1].pin = BUTTON2_PIN;
    buttons[2].pin = BUTTON3_PIN;
    buttons[3].pin = BUTTON4_PIN;

    for (int i = 0; i < 4; i++) {
        buttons[i].changeTime = 0;
        buttons[i].rawState = BUTTON_RELEASED;
        buttons[i].stableState = BUTTON_RELEASED;

        buttons[i].pressedEvent = false;
        buttons[i].releasedEvent = false;

        buttons[i].holdStart = 0;
        buttons[i].holdLatched = false;
    }
}

static void updateButton(Button &b, uint32_t now) {
    int raw = digitalRead(b.pin);

    if (raw != b.rawState) {
        b.rawState = raw;
        b.changeTime = now;
    }

    b.pressedEvent = false;
    b.releasedEvent = false;

    if ((now - b.changeTime) > button_debounce_time) {
        if (raw != b.stableState) {
            b.stableState = raw;

            if (raw == BUTTON_PRESSED)
                b.pressedEvent = true;
            else
                b.releasedEvent = true;
        }
    }
}

void pollButtons() {
    uint32_t now = millis();

    for (int i = 0; i < 4; i++)
        updateButton(buttons[i], now);
}

bool buttonHeld(uint8_t id, uint32_t ms)
{
    Button &b = buttons[id];

    if (b.stableState == BUTTON_PRESSED) {

        if (b.holdStart == 0)
            b.holdStart = millis();

        if (!b.holdLatched &&
            millis() - b.holdStart >= ms)
        {
            b.holdLatched = true;
            return true;
        }
    } else {
        b.holdStart = 0;
        b.holdLatched = false;
    }

    return false;
}

bool isButtonPressed(uint8_t id) {
    return buttons[id].pressedEvent;
}
 
bool isButtonReleased(uint8_t id) {
    return buttons[id].releasedEvent;
}
 
bool isButtonDown(uint8_t id) {
    return buttons[id].stableState == BUTTON_PRESSED;
}
 
bool isButtonUp(uint8_t id) {
    return buttons[id].stableState == BUTTON_RELEASED;
}
 
uint32_t buttonHeldMs(uint8_t id) {
    Button &b = buttons[id];
    if (b.stableState == BUTTON_PRESSED && b.holdStart != 0) {
        return millis() - b.holdStart;
    }
    return 0;
}

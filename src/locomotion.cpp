#include "locomotion.h"

uint8_t current_state = STATE_IDLE

void runMainSequence() {
    uint32_t now = millis();

    checkBatteryAlarm();
    
    if (!pid_settings_loaded) {
        loadPIDSettings();
    }


}
#include "mission.h"
#include "line_sensor.h"
#include "locomotion.h"
#include "display.h"

static int dist_encoder = 0;
static uint32_t mission_timer = 0;
static uint32_t current_mission = 0;


static void displayMissionInfo(uint32_t index, uint32_t encoder_left, uint32_t encoder_right, const MissionState &mission) {
    char line1_buf[24], line2_buf[24], line3_buf[24], line4_buf[24];
    snprintf(line1_buf, sizeof(line1_buf), "RUNNING MISSION: %d", index);

    switch (mission.condition) {
        case COND_DIST_GT: 
        snprintf(line2_buf, sizeof(line2_buf), "UNTIL DIST: %d", mission.condition_threshold); 
        break;
        case COND_ENCODER1_GT:
        snprintf(line2_buf, sizeof(line2_buf), "UNTIL L ENC: %d", mission.condition_threshold);
        break;
        case COND_ENCODER2_GT:
        snprintf(line2_buf, sizeof(line2_buf), "UNTIL R ENC: %d", mission.condition_threshold);
        break;
        default:
        snprintf(line2_buf, sizeof(line2_buf), "RUNNING");
    }

    snprintf(line3_buf, sizeof(line3_buf), "ENC L: %d ENC R: %d", encoder_left, encoder_right);
    snprintf(line4_buf, sizeof(line4_buf), "L SPD: %d R SPD: %d", mission.leftSpeed, mission.rightSpeed);
    
    displayOLED(line1_buf, line2_buf, line3_buf, line4_buf);
}

uint8_t MaskSensor(uint8_t maskLeft, uint8_t maskRight, MaskMode mode) {
    uint16_t sensorMask = ((uint16_t)maskRight << 8) | maskLeft;
    uint8_t result = 0;

    for (int i = 0; i < 16; i++) {
        if (mode == MASK_AND) {
            if (((sensorMask >> i) & 0x01) && getLineSensorDigital(i)) {
                result++;
            }
        } else if (mode == MASK_OR) {
            if (((sensorMask >> i) & 0x01) && getLineSensorDigital(i)) {
                return 1;
            }
        }
    }

    if (mode == MASK_AND) {
        return (result == __builtin_popcount(sensorMask)) ? 1 : 0;
    } else if (mode == MASK_OR) {
        return 0;
    }

    return 0;
}

void runStateLogic(const MissionState &s, bool justEntered) {
    if (justEntered) {
        // idk if this will be used
    }

    switch (s.mode) {
        case DIRECT_MOVE:
         displayMissionInfo(current_mission, readEncoder(1), readEncoder(2), s);
         moveMotors(s.leftSpeed, s.rightSpeed); 
         break;
        case PID: followLinePID(); break;
    }
}

bool checkStateObjective(const MissionState &s) {
    switch (s.condition) {
        case COND_ENCODER1_GT: return readEncoder(1) > s.condition_threshold; 
        case COND_ENCODER2_GT: return readEncoder(2) > s.condition_threshold;
        case COND_DIST_GT: return dist_encoder > s.condition_threshold;
        case COND_SENSOR_MASK: return MaskSensor(s.sensorLeft, s.sensorRight, s.maskMode);
        case COND_IMMEDIATE: return true;
    }
    return false;
}

MissionState missionStates[] = {
    {DIRECT_MOVE, 100, 100, 0, COND_SENSOR_MASK, 0, 0b11111111, 0b11111111, MASK_OR},
    {PID, 0, 0, 0, COND_SENSOR_MASK, 0, 0b00001111, 0b11110000, MASK_AND}
};

const int NUM_STATES = sizeof(missionStates) / sizeof(missionStates[0]);

void runMission() {
    dist_encoder = (readEncoder(1) + readEncoder(2)) / 2;

    if (current_mission >= NUM_STATES) {
        stopMotors();
        displayOLED("MISSION FINISHED", "BTN2=idle", "", "");
        return;
    }

    static int last_mission = -1;
    bool justEntered = (current_mission != last_mission);
    last_mission = current_mission;

    const MissionState &s = missionStates[current_mission];

    runStateLogic(s, justEntered);

    if (checkStateObjective(s)) {
        resetEncoder(1);
        resetEncoder(2);
        mission_timer = 0;

        current_mission++;
        last_mission = current_mission;
    }
}
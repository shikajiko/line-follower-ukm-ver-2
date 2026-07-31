#include "mission.h"
#include "line_sensor.h"
#include "locomotion.h"

static int dist_encoder = 0;
static uint32_t mission_timer = 0;
static uint32_t current_mission = 0;

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
        case DIRECT_MOVE: moveMotors(s.leftSpeed, s.rightSpeed); break;
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
    {DIRECT_MOVE, -50, 50, 0, COND_DIST_GT, 10, 0, 0, MASK_OR},
    {DIRECT_MOVE, 50, 50, 0, COND_DIST_GT, 10, 0, 0, MASK_OR}
};

const int NUM_STATES = sizeof(missionStates) / sizeof(missionStates[0]);

void runMission() {
    dist_encoder = (readEncoder(1) + readEncoder(2)) / 2;

    if (current_mission >= NUM_STATES) {
        stopMotors();
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
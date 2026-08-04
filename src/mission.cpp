#include "mission.h"
#include "line_sensor.h"
#include "locomotion.h"
#include "display.h"

static int dist_encoder = 0;
static int last_mission = -1;
static uint32_t mission_timer = 0;
static uint32_t current_mission = 0;

void resetMissionState() {
    last_mission = -1;
    current_mission = 0;
    mission_timer = 0;
    dist_encoder = 0;
    resetEncoder(1);
    resetEncoder(2);
}

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
        mission_timer = millis();
    }

    switch (s.mode) {
        case DIRECT_MOVE:
         displayMissionInfo(current_mission, readEncoder(1), readEncoder(2), s);
         moveMotors(s.leftSpeed, s.rightSpeed); 
         break;
        case PID_STRAIGHT: followLinePID(true); break;
        case PID: followLinePID(false); break;
    }
}

bool checkStateObjective(const MissionState &s) {
    switch (s.condition) {
        case COND_ENCODER1_GT: return readEncoder(1) > s.condition_threshold; 
        case COND_ENCODER2_GT: return readEncoder(2) > s.condition_threshold;
        case COND_DIST_GT: return dist_encoder > s.condition_threshold;
        case COND_SENSOR_MASK: return MaskSensor(s.sensorLeft, s.sensorRight, s.maskMode);
        case COND_TIMER: return (millis() - mission_timer) > s.condition_threshold;
        case COND_IMMEDIATE: return true;
    }
    return false;
}

// MODE, LSPEED, RSPEED, CONDITION, THRESHOLD, MASKLEFT, MASKRIGHT, MASKMODE, STOPMODE
MissionState missionStates[] = {
    {PID_STRAIGHT, 135, 140, COND_DIST_GT, 450, 0b11111100, 0b00111111, MASK_OR, STOP},
    {DIRECT_MOVE, -140, 140, COND_DIST_GT, 80, 0b00000000, 0b00011111, MASK_OR, STOP},
    {PID, 100, 100, COND_DIST_GT, 150, 0b11111100, 0b00111111, MASK_OR, NONE},
    {PID_STRAIGHT, 100, 100, COND_SENSOR_MASK, 300, 0b11000000, 0b00000000, MASK_OR, NONE},
    {PID, 100, 100, COND_DIST_GT, 1000, 0b11111100, 0b00111111, MASK_OR, NONE},
};

const int NUM_STATES = sizeof(missionStates) / sizeof(missionStates[0]);

void runMission() {
    dist_encoder = (readEncoder(1) + readEncoder(2)) / 2;

    if (current_mission >= NUM_STATES) {
        stopMotors();
        displayOLED("MISSION FINISHED", "BTN2=idle", "", "");
        return;
    }


    bool justEntered = (current_mission != last_mission);
    last_mission = current_mission;

    const MissionState &s = missionStates[current_mission];

    if (checkStateObjective(s) && !justEntered) {
        resetEncoder(1);
        resetEncoder(2);
        mission_timer = 0;

        last_mission = current_mission;
        current_mission++;
        dist_encoder = 0;

        if (s.stopMode == BRAKE) {
            brakeMotors();
        } else if (s.stopMode == STOP) {
            stopMotors();
        } 

        return;
    }

    runStateLogic(s, justEntered);
}
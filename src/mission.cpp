#include "mission.h"
#include "line_sensor.h"
#include "locomotion.h"
#include "display.h"
#include "encoder_calibrate.h"

static int dist_encoder = 0;
static int last_mission = -1;
static int start_index = 0;
static uint32_t mission_timer = 0;
static uint32_t current_mission = 0;
static uint32_t last_checkpoint = 0;


void resetMissionState() {
    last_mission = -1;
    current_mission = start_index;
    mission_timer = 0;
    dist_encoder = 0;
    resetEncoder(1);
    resetEncoder(2);
}

void resetCheckpoint() {
    last_checkpoint = 0;
}

void resetFromLastCheckpoint() {
    current_mission = last_checkpoint;
    last_mission = last_checkpoint;
    mission_timer = 0;
    dist_encoder = 0;
    resetEncoder(1);
    resetEncoder(2);
}

void updateStartIndex(int n) {
    start_index = n;
}

void setCurrentMission(int n) {
    if (n < NUM_STATES) {
        current_mission = n;
    }
}

static void displayMissionInfo(uint32_t index, uint32_t encoder_left, uint32_t encoder_right, const MissionState &mission) {
    char line1_buf[24], line2_buf[24], line3_buf[24], line4_buf[24];
    snprintf(line1_buf, sizeof(line1_buf), "RUNNING MISSION: %d", index);

    switch (mission.condition) {
        case JARAK: 
        snprintf(line2_buf, sizeof(line2_buf), "UNTIL DIST: %d", mission.condition_threshold); 
        break;
        case ENCODER_KIRI:
        snprintf(line2_buf, sizeof(line2_buf), "UNTIL L ENC: %d", mission.condition_threshold);
        break;
        case ENCODER_KANAN:
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
    if (s.is_checkpoint) {
        Serial.printf("this is checkpoint: %d\n", current_mission);
        last_checkpoint = current_mission;
    }
    
    if (justEntered) {
        mission_timer = millis();
    }

    bool is_inverted = false;

    switch (s.lineMode) {
        case LINE_WHITE: is_inverted = true; break;
        default: break;
    }

    switch (s.driveMode) {
        case DIRECT_MOVE:
         displayMissionInfo(current_mission, readEncoder(1), readEncoder(2), s);
         moveMotors(s.leftSpeed, s.rightSpeed); 
         break;
        case PID_STRAIGHT: followLinePID(true, is_inverted, s.leftSpeed); break;
        case PID: followLinePID(false, is_inverted, s.leftSpeed); break;
    }
}

bool checkStateObjective(const MissionState &s) {
    Serial.printf("threshold: %d\n", s.condition_threshold);
    Serial.printf("condition: %d\n", s.condition);
    int32_t result = convertEncoderToCm(dist_encoder);
    Serial.printf("current dist: %d\n", result);

    switch (s.condition) {
        case ENCODER_KIRI: return convertEncoderToCm(readEncoder(1)) >= s.condition_threshold; 
        case ENCODER_KANAN: return convertEncoderToCm(readEncoder(2)) >= s.condition_threshold;
        case JARAK:{
            Serial.printf("entered JARAK condition\n");
            return convertEncoderToCm(dist_encoder) >= s.condition_threshold;
        } 
        case SENSOR_MASK: return MaskSensor(s.sensorLeft, s.sensorRight, s.maskMode);
        case TIMER: return (millis() - mission_timer)/1000 >= s.condition_threshold;
        case SKIP: return true;
    }
    return false;
}

MissionState missionStates[MAX_MISSIONS] = {
    {LINE_BLACK, PID_STRAIGHT, 135, 140, JARAK, 450, 0b11111100, 0b00111111, MASK_OR, STOP, true},
    {LINE_BLACK, DIRECT_MOVE, -140, 140, JARAK, 80, 0b00000000, 0b00011111, MASK_OR, STOP, false},
    {LINE_BLACK, PID, 100, 100, JARAK, 150, 0b11111100, 0b00111111, MASK_OR, NONE, false},
    {LINE_BLACK, PID_STRAIGHT, 100, 100, SENSOR_MASK, 300, 0b11000000, 0b00000000, MASK_OR, NONE, false},
    {LINE_BLACK, PID, 100, 100, JARAK, 1000, 0b11111100, 0b00111111, MASK_OR, NONE, false},
};

int NUM_STATES = 5;

void runMission() {

    dist_encoder = (readEncoder(1) + readEncoder(2)) / 2;

    if (current_mission >= NUM_STATES) {
        stopMotors();
        displayOLED("MISSION FINISHED", "BTN2=idle", "", "");
        return;
    }

    if (last_checkpoint != 0 && last_checkpoint < NUM_STATES && current_mission < last_checkpoint) current_mission = last_checkpoint;
    Serial.printf("current mission is: %d\nlast_checkpoint: %d\n", current_mission, last_checkpoint);
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
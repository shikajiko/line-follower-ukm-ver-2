#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>

#define MAX_MISSIONS 200

enum LineMode { LINE_BLACK, LINE_WHITE };
enum DriveMode { DIRECT_MOVE, PID_STRAIGHT, PID };
enum ConditionType {
    COND_ENCODER1_GT,
    COND_ENCODER2_GT,
    COND_DIST_GT,
    COND_SENSOR_MASK,
    COND_IMMEDIATE,
    COND_TIMER
};
enum MaskMode { MASK_AND, MASK_OR };
enum StopMode { NONE, STOP, BRAKE };

struct MissionState {
    LineMode lineMode;
    DriveMode driveMode;
    int16_t leftSpeed;
    int16_t rightSpeed;
    ConditionType condition;
    int32_t condition_threshold;
    uint8_t sensorLeft;
    uint8_t sensorRight;
    MaskMode maskMode;
    StopMode stopMode;
};

extern MissionState missionStates[MAX_MISSIONS];
extern int NUM_STATES;

void resetMissionState();
void runMission();

#endif
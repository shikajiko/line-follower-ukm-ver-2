#ifndef MISSION_H
#define MISSION_H

#include "stdint.h"
 
enum MaskMode {
    MASK_AND,
    MASK_OR
};

enum DriveMode {
    DIRECT_MOVE,
    PID
};

enum ConditionType {
    COND_ENCODER1_GT,
    COND_ENCODER2_GT,
    COND_DIST_GT,
    COND_SENSOR_MASK,
    COND_IMMEDIATE
};

enum ActionFlags {
    ACT_NONE = 0,
    ACT_BRAKE = 1 << 0 
};

struct MissionState {
    DriveMode mode;
    int leftSpeed, rightSpeed;
    int timerMs;

    ConditionType condition;
    int32_t condition_threshold;

    uint8_t sensorRight;
    uint8_t sensorLeft;
    MaskMode maskMode;
};

uint8_t MaskSensor(uint8_t maskLeft, uint8_t maskRight, uint8_t mode);

void stateTimer(uint32_t timerMs);
void runStateLogic(const MissionState &s, bool justEntered);
bool checkStateObjective(const MissionState &s);
void runMission();
void resetMissionState();

#endif //mission.h
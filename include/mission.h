#ifndef MISSION_H
#define MISSION_H

#include "stdint.h"
 
enum MaskMode {
    MASK_AND,
    MASK_OR
};

enum DriveMode {
    DIRECT_MOVE,
    PID_STRAIGHT,
    PID
};

enum LineMode {
    LINE_BLACK, 
    LINE_WHITE
};

enum ConditionType {
    COND_ENCODER1_GT,
    COND_ENCODER2_GT,
    COND_DIST_GT,
    COND_SENSOR_MASK,
    COND_IMMEDIATE,
    COND_TIMER,
};

enum StopMode {
    NONE = 0,
    STOP = 1,
    BRAKE = 2
};

struct MissionState {
    LineMode lineMode;
    DriveMode driveMode;
    int leftSpeed, rightSpeed;

    ConditionType condition;
    int32_t condition_threshold;

    uint8_t sensorRight;
    uint8_t sensorLeft;

    MaskMode maskMode;
    StopMode stopMode;
};

uint8_t MaskSensor(uint8_t maskLeft, uint8_t maskRight, MaskMode mode);

void runStateLogic(const MissionState &s, bool justEntered);
bool checkStateObjective(const MissionState &s);
void runMission();
void resetMissionState();

#endif //mission.h
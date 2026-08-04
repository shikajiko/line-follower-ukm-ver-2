#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "stdint.h"

#define LINE_SENSOR_THRESHOLD 3600

void readLineSensors();
uint16_t readLineSensorChannel(uint8_t channel);

uint16_t getLineSensorRaw(uint8_t id);
uint8_t getLineSensorDigital(uint8_t id);
uint16_t getLineSensorThreshold(uint8_t id);

bool isLineDetected();

void ensureLineSensorThresholdDefaults();
void calibrateLineSensorsAuto();
void lineSensorCalibrationBegin();
void lineSensorCalibrationUpdate();
void lineSensorCalibrationEnd();

void saveCalibration();
void loadCalibration();
bool isCalibrationLoaded();

#endif // LINE_SENSOR_H
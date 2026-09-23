#pragma once

#include "stdint.h"
#define DEFAULT_ENCODER_VALUE 10


void calibrateEncoder();
void loadEncoderCalibration();
void saveEncoderCalibration(float new_val);
float getCalibratedEncoderValue();

int32_t convertEncoderToCm(int32_t encoderValue);

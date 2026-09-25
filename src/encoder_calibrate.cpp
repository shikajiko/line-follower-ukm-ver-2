#include "locomotion.h"
#include "encoder_calibrate.h"
#include <Preferences.h>
#include "display.h"

static float calibratedValue = DEFAULT_ENCODER_VALUE;
static int32_t calibratedMm = DEFAULT_ENCODER_VALUE * 10;
static Preferences encoder_preferences;
static bool encoder_cal_loaded;

void loadEncoderCalibration() {
    if (encoder_cal_loaded) return;

    encoder_preferences.begin("encoder", true);
    calibratedValue = encoder_preferences.getFloat("val", DEFAULT_ENCODER_VALUE);
    calibratedMm = calibratedValue * 10;
    encoder_cal_loaded = true;
}

void saveEncoderCalibration(float new_val) {
    encoder_preferences.begin("encoder", false);
    encoder_preferences.putFloat("val", new_val);
    encoder_preferences.end();

    calibratedValue = new_val;
}

float getCalibratedEncoderValue() {
    return calibratedValue;
}

void calibrateEncoder() {
    resetEncoder(1);
    resetEncoder(2);

    int32_t encoderLeft = readEncoder(1);
    int32_t encoderRight = readEncoder(2);

    while ((encoderLeft + encoderRight) / 2 < 250) {
        moveMotors(80, 80);
        encoderLeft = readEncoder(1);
        encoderRight = readEncoder(2);
    }

    brakeMotors();
}

int32_t convertEncoderToCm(int32_t encoderValue) {
    loadEncoderCalibration();
    return (encoderValue * calibratedMm) / 2500; 
}
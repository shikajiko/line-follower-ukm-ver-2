#include "IO.h"
#include "line_sensor.h"
#include "Arduino.h"
#include <Preferences.h>
#include <strings.h>

static uint16_t line_sensor_raw[16] = {0};
static uint8_t line_sensor_digital[16] = {0};
static uint16_t line_sensor_max[16] = {0};
static uint16_t line_sensor_min[16] = {0};
static uint16_t line_sensor_threshold[16] = {0};
static Preferences saved_calibration;
static bool is_calibration_loaded = false;

static void selectMUXChannel(uint8_t channel) {
  channel = channel & 0x0F;

  switch (channel) {
  case 0:
    digitalWrite(MUX_S0_PIN, LOW);
    digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, LOW);
    digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 1:
    digitalWrite(MUX_S0_PIN, HIGH);
    digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, LOW);
    digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 2:
    digitalWrite(MUX_S0_PIN, LOW);
    digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, LOW);
    digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 3:
    digitalWrite(MUX_S0_PIN, HIGH);
    digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, LOW);
    digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 4:
    digitalWrite(MUX_S0_PIN, LOW);
    digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, HIGH);
    digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 5:
    digitalWrite(MUX_S0_PIN, HIGH);
    digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, HIGH);
    digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 6:
    digitalWrite(MUX_S0_PIN, LOW);
    digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, HIGH);
    digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 7:
    digitalWrite(MUX_S0_PIN, HIGH);
    digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, HIGH);
    digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 8:
    digitalWrite(MUX_S0_PIN, LOW);
    digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, LOW);
    digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 9:
    digitalWrite(MUX_S0_PIN, HIGH);
    digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, LOW);
    digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 10:
    digitalWrite(MUX_S0_PIN, LOW);
    digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, LOW);
    digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 11:
    digitalWrite(MUX_S0_PIN, HIGH);
    digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, LOW);
    digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 12:
    digitalWrite(MUX_S0_PIN, LOW);
    digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, HIGH);
    digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 13:
    digitalWrite(MUX_S0_PIN, HIGH);
    digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, HIGH);
    digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 14:
    digitalWrite(MUX_S0_PIN, LOW);
    digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, HIGH);
    digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 15:
    digitalWrite(MUX_S0_PIN, HIGH);
    digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, HIGH);
    digitalWrite(MUX_S3_PIN, HIGH);
    break;
  }

  delayMicroseconds(5);
}

static uint16_t readLineSensor(uint8_t channel) {
  selectMUXChannel(channel);
  uint16_t raw = analogRead(MUX_ADC_PIN);
  line_sensor_raw[channel] = raw;
  return raw;
}

static uint8_t readLineSensorDigital(uint8_t channel) {
  uint16_t raw = line_sensor_raw[channel];
  uint8_t digital = (raw > line_sensor_threshold[channel]) ? 1 : 0;
  line_sensor_digital[channel] = digital;
  return digital;
}

void readLineSensors() {
  for (int i = 0; i < 16; i++) {
    readLineSensor(i);
    readLineSensorDigital(i);
  }
}

uint16_t readLineSensorChannel(uint8_t channel) {
  if (channel >= 16) {
    return 0;
  }
  return readLineSensor(channel);
}

uint16_t getLineSensorRaw(uint8_t id) {
  if (id >= 16) {
    return 0;
  }
  return line_sensor_raw[id];
}

uint8_t getLineSensorDigital(uint8_t id) {
  if (id >= 16) {
    return 0;
  }
  return line_sensor_digital[id];
}

uint16_t getLineSensorThreshold(uint8_t id) {
  if (id >= 16) {
    return 0;
  }
  return line_sensor_threshold[id];
}

bool isLineDetected() {
  for (int i = 0; i < 16; i++) {
    if (line_sensor_digital[i]) {
      return true;
    }
  }
  return false;
}

void ensureLineSensorThresholdDefaults() {
  for (int i = 0; i < 16; i++) {
    if (line_sensor_threshold[i] == 0) {
      line_sensor_threshold[i] = LINE_SENSOR_THRESHOLD;
    }
  }
}

void calibrateLineSensorsAuto() {
  ensureLineSensorThresholdDefaults();

  for (int i = 0; i < 16; i++) {
    if (line_sensor_max[i] == 0 && line_sensor_min[i] == 0) {
      line_sensor_threshold[i] = LINE_SENSOR_THRESHOLD;
    } else {
      line_sensor_threshold[i] =
          static_cast<uint16_t>((line_sensor_max[i] + line_sensor_min[i]) / 2);
    }
  }
}

void saveCalibration() {
  saved_calibration.begin("calibration", false);
  for (int i = 0; i < 16; i++) {
    saved_calibration.putFloat(("thresh" + String(i)).c_str(), line_sensor_threshold[i]);
  }
  saved_calibration.end();
}

void loadCalibration() {
  saved_calibration.begin("calibration", true);
  for (int i = 0; i < 16; i++) {
    line_sensor_threshold[i] = saved_calibration.getFloat(("thresh" + String(i)).c_str(), 0.);
  }
  saved_calibration.end();
  is_calibration_loaded = true;
}

void lineSensorCalibrationBegin() {
  for (int i = 0; i < 16; i++) {
    line_sensor_max[i] = 0;
    line_sensor_min[i] = 4095;
  }
}

bool isCalibrationLoaded() {
  return is_calibration_loaded;
}

void lineSensorCalibrationUpdate() {
  for (int i = 0; i < 16; i++) {
    if (line_sensor_raw[i] > line_sensor_max[i]) {
      line_sensor_max[i] = line_sensor_raw[i];
    }
    if (line_sensor_raw[i] < line_sensor_min[i]) {
      line_sensor_min[i] = line_sensor_raw[i];
    }
  }
}

void lineSensorCalibrationEnd() {
  for (int i = 0; i < 16; i++) {
    line_sensor_threshold[i] = (line_sensor_max[i] + line_sensor_min[i]) / 2;
  }
  saveCalibration();
}
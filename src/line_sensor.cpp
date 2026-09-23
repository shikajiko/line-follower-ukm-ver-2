#include "IO.h"
#include "line_sensor.h"
#include "Arduino.h"
#include <Preferences.h>
#include <strings.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

static uint16_t line_sensor_raw[16] = {0};
static uint8_t line_sensor_digital[16] = {0};
static uint16_t line_sensor_max[16] = {0};
static uint16_t line_sensor_min[16] = {0};
static uint16_t line_sensor_threshold[16] = {0};       
static uint16_t line_sensor_threshold_high[16] = {0};  
static uint16_t line_sensor_threshold_low[16] = {0};
static uint8_t line_sensor_debounce_count[16] = {0};
static Preferences saved_calibration;
static bool is_calibration_loaded = false;

static SemaphoreHandle_t line_sensor_mutex = nullptr;
static TaskHandle_t line_sensor_task_handle = nullptr;

static volatile bool line_sensor_calibration_active = false;

static inline bool lockLineSensors() {
  if (line_sensor_mutex == nullptr) {
    return true;
  }
  return xSemaphoreTake(line_sensor_mutex, portMAX_DELAY) == pdTRUE;
}
static inline void unlockLineSensors() {
  if (line_sensor_mutex != nullptr) {
    xSemaphoreGive(line_sensor_mutex);
  }
}

#define LINE_SENSOR_CAL_SAMPLE_WINDOW 3
static uint16_t line_sensor_cal_history[16][LINE_SENSOR_CAL_SAMPLE_WINDOW] = {{0}};
static uint8_t line_sensor_cal_hist_idx[16] = {0};
static uint8_t line_sensor_cal_hist_count[16] = {0};

#define LINE_SENSOR_HYSTERESIS_PCT 0.01f
#define LINE_SENSOR_HYSTERESIS_MAX_COUNTS 100
#define LINE_SENSOR_HYSTERESIS_MIN_COUNTS 20
#define LINE_SENSOR_HYSTERESIS_RANGE_FRACTION 0.05f
#define LINE_SENSOR_DEBOUNCE_N 1

#define LINE_SENSOR_MIN_VALID_RANGE 5
#define LINE_SENSOR_SCAN_PERIOD_MS 1

static uint16_t clampMargin(uint16_t margin) {
  if (margin > LINE_SENSOR_HYSTERESIS_MAX_COUNTS) {
    return LINE_SENSOR_HYSTERESIS_MAX_COUNTS;
  }
  if (margin < LINE_SENSOR_HYSTERESIS_MIN_COUNTS) {
    return LINE_SENSOR_HYSTERESIS_MIN_COUNTS;
  }
  return margin;
}

static uint16_t clampMarginToRange(uint16_t margin, uint16_t range) {
  if (margin > LINE_SENSOR_HYSTERESIS_MAX_COUNTS) {
    margin = LINE_SENSOR_HYSTERESIS_MAX_COUNTS;
  }
  if (margin < LINE_SENSOR_HYSTERESIS_MIN_COUNTS) {
    margin = LINE_SENSOR_HYSTERESIS_MIN_COUNTS;
  }
  uint16_t range_cap =
      static_cast<uint16_t>(range * LINE_SENSOR_HYSTERESIS_RANGE_FRACTION);
  if (margin > range_cap) {
    margin = range_cap;
  }
  return margin;
}

static void selectMUXChannel(uint8_t channel) {
  channel = channel & 0x0F;

  switch (channel) {
  case 0:
    digitalWrite(MUX_S0_PIN, LOW); digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, LOW); digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 1:
    digitalWrite(MUX_S0_PIN, HIGH); digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, LOW); digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 2:
    digitalWrite(MUX_S0_PIN, LOW); digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, LOW); digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 3:
    digitalWrite(MUX_S0_PIN, HIGH); digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, LOW); digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 4:
    digitalWrite(MUX_S0_PIN, LOW); digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, HIGH); digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 5:
    digitalWrite(MUX_S0_PIN, HIGH); digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, HIGH); digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 6:
    digitalWrite(MUX_S0_PIN, LOW); digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, HIGH); digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 7:
    digitalWrite(MUX_S0_PIN, HIGH); digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, HIGH); digitalWrite(MUX_S3_PIN, LOW);
    break;
  case 8:
    digitalWrite(MUX_S0_PIN, LOW); digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, LOW); digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 9:
    digitalWrite(MUX_S0_PIN, HIGH); digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, LOW); digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 10:
    digitalWrite(MUX_S0_PIN, LOW); digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, LOW); digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 11:
    digitalWrite(MUX_S0_PIN, HIGH); digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, LOW); digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 12:
    digitalWrite(MUX_S0_PIN, LOW); digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, HIGH); digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 13:
    digitalWrite(MUX_S0_PIN, HIGH); digitalWrite(MUX_S1_PIN, LOW);
    digitalWrite(MUX_S2_PIN, HIGH); digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 14:
    digitalWrite(MUX_S0_PIN, LOW); digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, HIGH); digitalWrite(MUX_S3_PIN, HIGH);
    break;
  case 15:
    digitalWrite(MUX_S0_PIN, HIGH); digitalWrite(MUX_S1_PIN, HIGH);
    digitalWrite(MUX_S2_PIN, HIGH); digitalWrite(MUX_S3_PIN, HIGH);
    break;
  }

  delayMicroseconds(3);
}

static uint16_t readLineSensor(uint8_t channel) {
  selectMUXChannel(channel);
  uint16_t raw = analogRead(MUX_ADC_PIN);
  line_sensor_raw[channel] = raw;
  return raw;
}

static uint8_t readLineSensorDigital(uint8_t channel) {
  uint16_t raw = line_sensor_raw[channel];
  uint8_t prev = line_sensor_digital[channel];
  uint8_t candidate = prev;

  if (prev == 0 && raw > line_sensor_threshold_high[channel]) {
    candidate = 1;
  } else if (prev == 1 && raw < line_sensor_threshold_low[channel]) {
    candidate = 0;
  }

  if (candidate != prev) {
    line_sensor_debounce_count[channel]++;
    if (line_sensor_debounce_count[channel] >= LINE_SENSOR_DEBOUNCE_N) {
      line_sensor_digital[channel] = candidate;
      line_sensor_debounce_count[channel] = 0;
    }
  } else {
    line_sensor_debounce_count[channel] = 0;
  }

  return line_sensor_digital[channel];
}

static void applyHysteresisFromMidpoint(uint8_t i, uint16_t margin) {
  line_sensor_threshold_high[i] = line_sensor_threshold[i] + margin;
  line_sensor_threshold_low[i] =
      (line_sensor_threshold[i] > margin) ? (line_sensor_threshold[i] - margin) : 0;
}

void lineSensorCalibrationUpdate() {
  for (int i = 0; i < 16; i++) {
    uint8_t idx = line_sensor_cal_hist_idx[i];
    line_sensor_cal_history[i][idx] = line_sensor_raw[i];
    line_sensor_cal_hist_idx[i] = (idx + 1) % LINE_SENSOR_CAL_SAMPLE_WINDOW;
    if (line_sensor_cal_hist_count[i] < LINE_SENSOR_CAL_SAMPLE_WINDOW) {
      line_sensor_cal_hist_count[i]++;
    }

    uint32_t sum = 0;
    for (uint8_t k = 0; k < line_sensor_cal_hist_count[i]; k++) {
      sum += line_sensor_cal_history[i][k];
    }
    uint16_t smoothed =
        static_cast<uint16_t>(sum / line_sensor_cal_hist_count[i]);

    if (smoothed > line_sensor_max[i]) {
      line_sensor_max[i] = smoothed;
    }
    if (smoothed < line_sensor_min[i]) {
      line_sensor_min[i] = smoothed;
    }
  }
}


static void scanAllChannelsLocked() {
  for (int i = 0; i < 16; i++) {
    readLineSensor(i);
    readLineSensorDigital(i);
  }
  if (line_sensor_calibration_active) {
    lineSensorCalibrationUpdate();
  }
}

static void lineSensorTask(void *pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(LINE_SENSOR_SCAN_PERIOD_MS);
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    if (xSemaphoreTake(line_sensor_mutex, portMAX_DELAY) == pdTRUE) {
      scanAllChannelsLocked();
      xSemaphoreGive(line_sensor_mutex);
    }
    vTaskDelayUntil(&lastWake, period);
  }
}

void startLineSensorTask() {
  if (line_sensor_mutex == nullptr) {
    line_sensor_mutex = xSemaphoreCreateMutex();
  }
  if (line_sensor_task_handle == nullptr) {
    xTaskCreatePinnedToCore(
        lineSensorTask,
        "LineSensorTask",
        4096,
        nullptr,
        2,     
        &line_sensor_task_handle,
        0       
    );
  }
}

void readLineSensors() {
  if (lockLineSensors()) {
    scanAllChannelsLocked();
    unlockLineSensors();
  }
}

uint16_t readLineSensorChannel(uint8_t channel) {
  if (channel >= 16) {
    return 0;
  }
  uint16_t val = 0;
  if (lockLineSensors()) {
    val = readLineSensor(channel);
    unlockLineSensors();
  }
  return val;
}

uint16_t getLineSensorRaw(uint8_t id) {
  if (id >= 16) return 0;
  uint16_t val = 0;
  if (lockLineSensors()) {
    val = line_sensor_raw[id];
    unlockLineSensors();
  }
  return val;
}

uint8_t getLineSensorDigital(uint8_t id) {
  if (id >= 16) return 0;
  uint8_t val = 0;
  if (lockLineSensors()) {
    val = line_sensor_digital[id];
    unlockLineSensors();
  }
  return val;
}

uint16_t getLineSensorThreshold(uint8_t id) {
  if (id >= 16) return 0;
  uint16_t val = 0;
  if (lockLineSensors()) {
    val = line_sensor_threshold[id];
    unlockLineSensors();
  }
  return val;
}

uint16_t getLineSensorMax(uint8_t id) {
  if (id >= 16) return 0;
  uint16_t val = 0;
  if (lockLineSensors()) {
    val = line_sensor_max[id];
    unlockLineSensors();
  }
  return val;
}

uint16_t getLineSensorMin(uint8_t id) {
  if (id >= 16) return 0;
  uint16_t val = 0;
  if (lockLineSensors()) {
    val = line_sensor_min[id];
    unlockLineSensors();
  }
  return val;
}

void getLineSensorSnapshot(uint16_t rawOut[16], uint8_t digitalOut[16]) {
  if (lockLineSensors()) {
    memcpy(rawOut, line_sensor_raw, sizeof(line_sensor_raw));
    memcpy(digitalOut, line_sensor_digital, sizeof(line_sensor_digital));
    unlockLineSensors();
  }
}

void printLineSensorCalibration() {
  uint16_t min_s[16], max_s[16], thr_s[16], thrH_s[16], thrL_s[16];
  if (lockLineSensors()) {
    memcpy(min_s, line_sensor_min, sizeof(min_s));
    memcpy(max_s, line_sensor_max, sizeof(max_s));
    memcpy(thr_s, line_sensor_threshold, sizeof(thr_s));
    memcpy(thrH_s, line_sensor_threshold_high, sizeof(thrH_s));
    memcpy(thrL_s, line_sensor_threshold_low, sizeof(thrL_s));
    unlockLineSensors();
  }

  Serial.println(F("ch\tmin\tmax\trange\tmid\thigh\tlow"));
  for (int i = 0; i < 16; i++) {
    uint16_t range = (max_s[i] > min_s[i]) ? (max_s[i] - min_s[i]) : 0;
    Serial.print(i); Serial.print('\t');
    Serial.print(min_s[i]); Serial.print('\t');
    Serial.print(max_s[i]); Serial.print('\t');
    Serial.print(range); Serial.print('\t');
    Serial.print(thr_s[i]); Serial.print('\t');
    Serial.print(thrH_s[i]); Serial.print('\t');
    Serial.print(thrL_s[i]);
    if (range < LINE_SENSOR_MIN_VALID_RANGE) {
      Serial.print(F("\t<-- LOW RANGE, check sensor/alignment"));
    }
    Serial.println();
  }
}

bool isLineDetected() {
  bool detected = false;
  if (lockLineSensors()) {
    for (int i = 0; i < 16; i++) {
      if (line_sensor_digital[i]) {
        detected = true;
        break;
      }
    }
    unlockLineSensors();
  }
  return detected;
}

void ensureLineSensorThresholdDefaults() {
  if (!lockLineSensors()) return;
  for (int i = 0; i < 16; i++) {
    if (line_sensor_threshold[i] == 0) {
      line_sensor_threshold[i] = LINE_SENSOR_THRESHOLD;
    }
    if (line_sensor_threshold_high[i] == 0 && line_sensor_threshold_low[i] == 0) {
      uint16_t margin = clampMargin(
          static_cast<uint16_t>(line_sensor_threshold[i] * LINE_SENSOR_HYSTERESIS_PCT));
      applyHysteresisFromMidpoint(i, margin);
    }
  }
  unlockLineSensors();
}

void calibrateLineSensorsAuto() {
  ensureLineSensorThresholdDefaults();

  if (is_calibration_loaded) {
    return;
  }

  if (!lockLineSensors()) return;
  for (int i = 0; i < 16; i++) {
    if (line_sensor_max[i] == 0 && line_sensor_min[i] == 0) {
      line_sensor_threshold[i] = LINE_SENSOR_THRESHOLD;
      uint16_t margin = clampMargin(
          static_cast<uint16_t>(line_sensor_threshold[i] * LINE_SENSOR_HYSTERESIS_PCT));
      applyHysteresisFromMidpoint(i, margin);
    } else {
      uint16_t range = line_sensor_max[i] - line_sensor_min[i];
      if (range < LINE_SENSOR_MIN_VALID_RANGE) {
        line_sensor_threshold[i] = LINE_SENSOR_THRESHOLD;
        uint16_t margin = clampMargin(
            static_cast<uint16_t>(line_sensor_threshold[i] * LINE_SENSOR_HYSTERESIS_PCT));
        applyHysteresisFromMidpoint(i, margin);
      } else {
        uint16_t margin = clampMarginToRange(
            static_cast<uint16_t>(range * LINE_SENSOR_HYSTERESIS_PCT), range);
        line_sensor_threshold[i] =
            static_cast<uint16_t>((line_sensor_max[i] + line_sensor_min[i]) / 2);
        line_sensor_threshold_high[i] = line_sensor_threshold[i] + margin;
        line_sensor_threshold_low[i] =
            (line_sensor_threshold[i] > margin) ? (line_sensor_threshold[i] - margin) : 0;
      }
    }
  }
  unlockLineSensors();
}

void saveCalibration() {
  uint16_t th[16], thh[16], thl[16];
  if (lockLineSensors()) {
    memcpy(th, line_sensor_threshold, sizeof(th));
    memcpy(thh, line_sensor_threshold_high, sizeof(thh));
    memcpy(thl, line_sensor_threshold_low, sizeof(thl));
    unlockLineSensors();
  }

  saved_calibration.begin("calibration", false);
  for (int i = 0; i < 16; i++) {
    saved_calibration.putFloat(("thresh" + String(i)).c_str(), th[i]);
    saved_calibration.putFloat(("threshH" + String(i)).c_str(), thh[i]);
    saved_calibration.putFloat(("threshL" + String(i)).c_str(), thl[i]);
  }
  saved_calibration.end();
}

void loadCalibration() {
  uint16_t th[16], thh[16], thl[16];

  saved_calibration.begin("calibration", true);
  for (int i = 0; i < 16; i++) {
    th[i] = saved_calibration.getFloat(("thresh" + String(i)).c_str(), 0.);
    thh[i] = saved_calibration.getFloat(("threshH" + String(i)).c_str(), 0.);
    thl[i] = saved_calibration.getFloat(("threshL" + String(i)).c_str(), 0.);
  }
  saved_calibration.end();

  for (int i = 0; i < 16; i++) {
    if (thh[i] == 0 && thl[i] == 0 && th[i] != 0) {
      uint16_t margin = clampMargin(static_cast<uint16_t>(th[i] * LINE_SENSOR_HYSTERESIS_PCT));
      thh[i] = th[i] + margin;
      thl[i] = (th[i] > margin) ? (th[i] - margin) : 0;
    }
  }

  if (lockLineSensors()) {
    memcpy(line_sensor_threshold, th, sizeof(th));
    memcpy(line_sensor_threshold_high, thh, sizeof(thh));
    memcpy(line_sensor_threshold_low, thl, sizeof(thl));
    unlockLineSensors();
  }

  is_calibration_loaded = true;
}

void lineSensorCalibrationBegin() {
  if (!lockLineSensors()) return;
  for (int i = 0; i < 16; i++) {
    line_sensor_max[i] = 0;
    line_sensor_min[i] = 4095;
    line_sensor_debounce_count[i] = 0;
    line_sensor_cal_hist_idx[i] = 0;
    line_sensor_cal_hist_count[i] = 0;
  }
  line_sensor_calibration_active = true;  
  unlockLineSensors();
}

bool isCalibrationLoaded() {
  return is_calibration_loaded;
}

void lineSensorCalibrationEnd() {
  if (lockLineSensors()) {
    line_sensor_calibration_active = false;  
    for (int i = 0; i < 16; i++) {
      uint16_t range = line_sensor_max[i] - line_sensor_min[i];
      if (range < LINE_SENSOR_MIN_VALID_RANGE) {
        Serial.print(F("WARNING: line sensor channel "));
        Serial.print(i);
        Serial.print(F(" calibration range too small ("));
        Serial.print(range);
        Serial.println(F("), keeping previous threshold"));
        continue;
      }
      uint16_t mid = (line_sensor_max[i] + line_sensor_min[i]) / 2;
      uint16_t margin = clampMarginToRange(
          static_cast<uint16_t>(range * LINE_SENSOR_HYSTERESIS_PCT), range);
      line_sensor_threshold[i] = mid;
      line_sensor_threshold_high[i] = mid + margin;
      line_sensor_threshold_low[i] = (mid > margin) ? (mid - margin) : 0;
    }
    unlockLineSensors();
  }
  saveCalibration();
}
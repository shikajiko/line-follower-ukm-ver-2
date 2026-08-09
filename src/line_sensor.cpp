#include "IO.h"
#include "line_sensor.h"
#include "Arduino.h"
#include <Preferences.h>
#include <strings.h>

static uint16_t line_sensor_raw[16] = {0};
static uint8_t line_sensor_digital[16] = {0};
static uint16_t line_sensor_max[16] = {0};
static uint16_t line_sensor_min[16] = {0};
static uint16_t line_sensor_threshold[16] = {0};       // kept for compatibility (midpoint, used by getLineSensorThreshold())
static uint16_t line_sensor_threshold_high[16] = {0};  // turn-ON bound (hysteresis)
static uint16_t line_sensor_threshold_low[16] = {0};   // turn-OFF bound (hysteresis)
static uint8_t line_sensor_debounce_count[16] = {0};
static Preferences saved_calibration;
static bool is_calibration_loaded = false;

// Tunables -------------------------------------------------------------
// Fraction of (max-min) calibration range used as the +/- margin around
// the midpoint. Widened from 0.03 to 0.08: the old value was tuned right
// at the edge of noise tolerance, which meant normal sensor jitter could
// sit close enough to the midpoint to cause missed / delayed detections.
// A wider band swallows more jitter at a small cost in sensitivity.
//   - If you start seeing real line hits fail to register, raise this
//     further. If you see false triggers on a clean surface, lower it.
#define LINE_SENSOR_HYSTERESIS_PCT 0.08f

// Hard cap on the margin in raw ADC counts, regardless of range. Raised
// from 60 to 100 so the larger percentage above can actually take effect
// on sensors with a wide calibrated range, instead of being clipped back
// down to the old ceiling.
#define LINE_SENSOR_HYSTERESIS_MAX_COUNTS 100

// Hard floor on the margin in raw ADC counts, used only for sensors with
// no calibrated range to compare against (defaults / never-calibrated).
#define LINE_SENSOR_HYSTERESIS_MIN_COUNTS 20

// Cap on the margin as a fraction of a sensor's OWN calibrated range
// (max-min). This matters for lower-contrast sensors (e.g. a center
// sensor with less black/white separation than the outer ones): if the
// margin is allowed to eat too much of a small range, the "high" trip
// point ends up right next to the one-time calibration-run max, which
// real-world readings then rarely reach again -> the sensor gets stuck
// reporting 0 forever. Capping relative to each sensor's own range keeps
// real headroom below max regardless of how narrow that sensor's range is.
#define LINE_SENSOR_HYSTERESIS_RANGE_FRACTION 0.30f

// Number of consecutive same-direction reads required before the digital
// state actually flips. 1 = hysteresis only, no added delay. This is
// already the fastest possible setting — the state flips the instant the
// threshold is crossed. Do NOT raise this to try to go faster; raising it
// only adds latency. Only raise it (2-3) if flicker persists after
// widening the hysteresis margin above, and you can tolerate the delay.
#define LINE_SENSOR_DEBOUNCE_N 1
// ------------------------------------------------------------------------

// Used when there's no measured range to compare against (defaults /
// never-calibrated channels). Flat floor/ceiling only.
static uint16_t clampMargin(uint16_t margin) {
  if (margin > LINE_SENSOR_HYSTERESIS_MAX_COUNTS) {
    return LINE_SENSOR_HYSTERESIS_MAX_COUNTS;
  }
  if (margin < LINE_SENSOR_HYSTERESIS_MIN_COUNTS) {
    return LINE_SENSOR_HYSTERESIS_MIN_COUNTS;
  }
  return margin;
}

// Used when a real per-sensor calibrated range is available. Applies the
// normal floor/ceiling, then caps to a fraction of THIS sensor's own
// range so the trip point never crowds out that sensor's actual max/min.
// Deliberately does NOT enforce the flat floor if the range is too small
// to support it — a smaller-than-usual margin that still works beats a
// "safe" one that never trips.
static uint16_t clampMarginToRange(uint16_t margin, uint16_t range) {
  if (margin > LINE_SENSOR_HYSTERESIS_MAX_COUNTS) {
    margin = LINE_SENSOR_HYSTERESIS_MAX_COUNTS;
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

  // Trimmed from 5us to 3us: shaves per-channel latency across all 16
  // channels every scan cycle. Still comfortably above typical analog
  // MUX settle times. If you see noisy/erratic readings after this
  // change, that's the first thing to revert.
  delayMicroseconds(3);
}

static uint16_t readLineSensor(uint8_t channel) {
  selectMUXChannel(channel);
  uint16_t raw = analogRead(MUX_ADC_PIN);
  line_sensor_raw[channel] = raw;
  return raw;
}

// Applies hysteresis (dual threshold) + debounce to avoid 0/1 jitter
// when the raw reading sits close to the calibrated midpoint.
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
    // Reading agrees with current state again; reset the debounce counter.
    line_sensor_debounce_count[channel] = 0;
  }

  return line_sensor_digital[channel];
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

// Recomputes the high/low hysteresis bounds from the current midpoint
// threshold using a fixed absolute margin (used when no min/max range
// is available, e.g. defaults).
static void applyHysteresisFromMidpoint(uint8_t i, uint16_t margin) {
  line_sensor_threshold_high[i] = line_sensor_threshold[i] + margin;
  line_sensor_threshold_low[i] =
      (line_sensor_threshold[i] > margin) ? (line_sensor_threshold[i] - margin) : 0;
}

void ensureLineSensorThresholdDefaults() {
  for (int i = 0; i < 16; i++) {
    if (line_sensor_threshold[i] == 0) {
      line_sensor_threshold[i] = LINE_SENSOR_THRESHOLD;
    }
    if (line_sensor_threshold_high[i] == 0 && line_sensor_threshold_low[i] == 0) {
      // Default margin: small fraction of the default threshold value, capped/floored.
      uint16_t margin = clampMargin(
          static_cast<uint16_t>(line_sensor_threshold[i] * LINE_SENSOR_HYSTERESIS_PCT));
      applyHysteresisFromMidpoint(i, margin);
    }
  }
}

void calibrateLineSensorsAuto() {
  ensureLineSensorThresholdDefaults();

  for (int i = 0; i < 16; i++) {
    if (line_sensor_max[i] == 0 && line_sensor_min[i] == 0) {
      line_sensor_threshold[i] = LINE_SENSOR_THRESHOLD;
      uint16_t margin = clampMargin(
          static_cast<uint16_t>(line_sensor_threshold[i] * LINE_SENSOR_HYSTERESIS_PCT));
      applyHysteresisFromMidpoint(i, margin);
    } else {
      uint16_t range = line_sensor_max[i] - line_sensor_min[i];
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

void saveCalibration() {
  saved_calibration.begin("calibration", false);
  for (int i = 0; i < 16; i++) {
    saved_calibration.putFloat(("thresh" + String(i)).c_str(), line_sensor_threshold[i]);
    saved_calibration.putFloat(("threshH" + String(i)).c_str(), line_sensor_threshold_high[i]);
    saved_calibration.putFloat(("threshL" + String(i)).c_str(), line_sensor_threshold_low[i]);
  }
  saved_calibration.end();
}

void loadCalibration() {
  saved_calibration.begin("calibration", true);
  for (int i = 0; i < 16; i++) {
    line_sensor_threshold[i] = saved_calibration.getFloat(("thresh" + String(i)).c_str(), 0.);
    line_sensor_threshold_high[i] =
        saved_calibration.getFloat(("threshH" + String(i)).c_str(), 0.);
    line_sensor_threshold_low[i] =
        saved_calibration.getFloat(("threshL" + String(i)).c_str(), 0.);
  }
  saved_calibration.end();

  // Backward compatibility: if this NVS blob was saved by the old version
  // of the code (no threshH/threshL keys), high/low will still be 0 here.
  // Derive them from the midpoint so hysteresis still works immediately.
  for (int i = 0; i < 16; i++) {
    if (line_sensor_threshold_high[i] == 0 && line_sensor_threshold_low[i] == 0 &&
        line_sensor_threshold[i] != 0) {
      uint16_t margin = clampMargin(
          static_cast<uint16_t>(line_sensor_threshold[i] * LINE_SENSOR_HYSTERESIS_PCT));
      applyHysteresisFromMidpoint(i, margin);
    }
  }

  is_calibration_loaded = true;
}

void lineSensorCalibrationBegin() {
  for (int i = 0; i < 16; i++) {
    line_sensor_max[i] = 0;
    line_sensor_min[i] = 4095;
    line_sensor_debounce_count[i] = 0;
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
    uint16_t mid = (line_sensor_max[i] + line_sensor_min[i]) / 2;
    uint16_t range = line_sensor_max[i] - line_sensor_min[i];
    uint16_t margin = clampMarginToRange(
        static_cast<uint16_t>(range * LINE_SENSOR_HYSTERESIS_PCT), range);

    line_sensor_threshold[i] = mid;
    line_sensor_threshold_high[i] = mid + margin;
    line_sensor_threshold_low[i] = (mid > margin) ? (mid - margin) : 0;
  }
  saveCalibration();
}
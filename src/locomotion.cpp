#include "locomotion.h"
#include "IO.h"
#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <driver/pcnt.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

uint8_t current_state = STATE_IDLE;

// ========== PID CONTROLLER =============

// PID Controller for line following
PIDController pid_line_following;
bool pid_enabled = true;
float pid_line_position = 0.0f;
float prev_pid_correction = 0.0f;
bool line_detected = false;
static bool pid_settings_loaded = false;
bool pid_menu_active = false;
static uint8_t pid_menu_item = 0;
uint32_t pid_last_update_ms = 0;
static Preferences pid_preferences;

// PID parameters (adjustable during runtime)


void initPIDController(PIDController *pid, float Kp, float Ki, float Kd,
                        float integral_limit, float output_limit) {
  if (pid == nullptr) {
    return;
  }

  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
  pid->integral_limit = integral_limit;
  pid->output_limit = output_limit;
}

float calculatePID(PIDController *pid, float setpoint, float current_value,
                    float dt) {
  if (pid == nullptr || dt <= 0.0f) {
    return 0.0f;
  }

  const float error = setpoint - current_value;
  pid->integral += error * dt;

  if (pid->integral > pid->integral_limit) {
    pid->integral = pid->integral_limit;
  } else if (pid->integral < -pid->integral_limit) {
    pid->integral = -pid->integral_limit;
  }

  const float derivative = (error - pid->last_error) / dt;
  pid->last_error = error;

  float output =
      (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

  if (output > pid->output_limit) {
    output = pid->output_limit;
  } else if (output < -pid->output_limit) {
    output = -pid->output_limit;
  }

  return output;
}

void resetPID(PIDController *pid) {
  if (pid == nullptr) {
    return;
  }

  pid->integral = 0.0f;
  pid->last_error = 0.0f;
}

static void syncPIDController() {
  initPIDController(&pid_line_following, pid_current_Kp, pid_current_Ki,
                     pid_current_Kd, pid_integral_limit, pid_output_limit);
}

static void savePIDSettings() {
  pid_preferences.begin("pidline", false);
  pid_preferences.putFloat("kp", pid_current_Kp);
  pid_preferences.putFloat("ki", pid_current_Ki);
  pid_preferences.putFloat("kd", pid_current_Kd);
  pid_preferences.putFloat("base", pid_base_speed);
  pid_preferences.end();
}

void loadPIDSettings() {
  if (pid_settings_loaded) {
    return;
  }

  ensureLineSensorThresholdDefaults();

  pid_preferences.begin("pidline", true);
  pid_current_Kp = pid_preferences.getFloat("kp", pid_current_Kp);
  pid_current_Ki = pid_preferences.getFloat("ki", pid_current_Ki);
  pid_current_Kd = pid_preferences.getFloat("kd", pid_current_Kd);
  pid_base_speed = pid_preferences.getFloat("base", pid_base_speed);
  pid_preferences.end();

  if (pid_current_Kp < 0.0f) {
    pid_current_Kp = 0.0f;
  }
  if (pid_current_Ki < 0.0f) {
    pid_current_Ki = 0.0f;
  }
  if (pid_current_Kd < 0.0f) {
    pid_current_Kd = 0.0f;
  }
  if (pid_base_speed < 0.0f) {
    pid_base_speed = 0.0f;
  } else if (pid_base_speed > 255.0f) {
    pid_base_speed = 255.0f;
  }

  syncPIDController();
  resetPID(&pid_line_following);
  pid_settings_loaded = true;
}

void resetPIDValues() {
  pid_current_Kp = DEFAULT_KP;
  pid_current_Ki = DEFAULT_KI;
  pid_current_Kd = DEFAULT_KD;
  pid_base_speed = DEFAULT_PID_BASE_SPEED;
  syncPIDController();
  resetPID(&pid_line_following);
  savePIDSettings();
}

float calculateLinePosition() {
  ensureLineSensorThresholdDefaults();
  int left_weight = 0;
  int right_weight = 0;

  float weighted_sum = 0.0f;
  float weight_total = 0.0f;

  for (int i = 0; i < 16; i++) {
    const uint8_t active = readLineSensorDigital(i);
    if (active) {
      const float strength = static_cast<float>(line_sensor_raw[i]) -
                              static_cast<float>(line_sensor_threshold[i]);
      const float weight = (strength > 0.0f) ? strength : 1.0f;
      weighted_sum += static_cast<float>(i) * weight;
      weight_total += weight;
    }
  }

  if (weight_total <= 0.0f) {
    line_detected = false;
    return pid_line_position;
  }

  line_detected = true;
  pid_line_position = weighted_sum / weight_total;
  return pid_line_position;
}



static void displayPIDDebug(const float line_pos, const float correction, int16_t right_speed, int16_t left_speed, bool is_line_detected) {
  static uint32_t debug_cycle_start_ms = 0;
  if (debug_cycle_start_ms == 0) {
    debug_cycle_start_ms = millis();
  }

  char bitmask_buf[17];
  for (int i = 0; i < 16; i++) {
    bitmask_buf[i] = line_sensor_digital[i] ? '1' : '0';
  }
  bitmask_buf[16] = '\0';

  int active_cnt = 0;
  for (int i = 0; i < 16; i++) {
    if (line_sensor_digital[i]) {
      active_cnt++;
    }
  }

  char line2_buf[24];
  if (is_line_detected) {
    snprintf(line2_buf, sizeof(line2_buf), "POS: %.1f PID: %.1f", line_pos, correction);
  } else {
    snprintf(line2_buf, sizeof(line2_buf), "NO LINE DETECTED");
  }

  // Rotate through 4 groups of 4 channels every 800ms so raw values for
  // all 16 sensors are visible over time.
  char line3_buf[24];
  snprintf(line3_buf, sizeof(line3_buf), "RIGHT: %d LEFT: %d", right_speed, left_speed);

  if (!battery_owns_display) {
    displayOLED(bitmask_buf, line2_buf, line3_buf, "");
  }
}

static uint32_t line_lost_ms;

void followLinePID(float base_speed, float max_speed_diff) {
  static uint8_t prev_cmd = 0;
  static uint32_t turn_dir_locked_until_ms = 0;
  static uint8_t locked_dir = 0; // 0 = none, 1 = left, 2 = right
  bool should_turn_left = false;
  bool should_turn_right = false;

  if (!pid_enabled) {
    stopMotors();
    return;
  }

  if (base_speed < 0.0f) {
    base_speed = 0.0f;
  } else if (base_speed > 255.0f) {
    base_speed = 255.0f;
  }

  if (max_speed_diff < 0.0f) {
    max_speed_diff = 0.0f;
  }

  const uint32_t now = millis();
  float dt = 0.02f;
  if (pid_last_update_ms != 0) {
    dt = (now - pid_last_update_ms) / 1000.0f;
    if (dt <= 0.0f) {
      dt = 0.02f;
    }
  }
  pid_last_update_ms = now;

  const float line_position = calculateLinePosition(should_turn_left, should_turn_right);

  float correction = 0;

  pid_line_following.output_limit = max_speed_diff;

  if (!line_detected) {
    if (line_lost_ms == 0) {
      line_lost_ms = now;
    }
    if (now - line_lost_ms > 1000) {
      stopMotors();
      resetPID(&pid_line_following);
      line_lost_ms = 0;
      return;

    }
    correction = prev_pid_correction;
  } else {
    line_lost_ms = 0;
    correction =
      calculatePID(&pid_line_following, DEFAULT_PID_SETPOINT, line_position, dt);
      prev_pid_correction = correction;
  }

  int16_t left_speed = static_cast<int16_t>(base_speed - correction);
  int16_t right_speed = static_cast<int16_t>(base_speed + correction);

  if (left_speed > 255) {
    left_speed = 255;
  } else if (left_speed < -255) {
    left_speed = -255; 
  }
  if (right_speed > 255) {
    right_speed = 255;
  } else if (right_speed < -255) {
    right_speed = -255;
  }

  if (should_turn_left) {
    Serial.write("LEFT\n");
    turnLeft();
    return;
    
  } else if (should_turn_right) {
    Serial.write("RIGHT\n");
    turnRight();
    return;
  } 

  Serial.write("STRAIGHT\n");

  setMotor(1, left_speed);
  setMotor(2, right_speed);

  displayPIDDebug(line_position, correction, right_speed, left_speed, line_detected);
}

// ============== LINE SENSOR DEBUG DISPLAY =============
// Builds a compact 4-line debug view of the 16 line sensors:
//   line1: current mode label
//   line2: 16-char bitmask of which sensors are ON/OFF (1=detected)
//   line3: position + how many sensors are active
//   line4: raw ADC values for 4 channels at a time, cycling every ~800ms
//          so all 16 channels get shown in rotation without cluttering
//          the small OLED.

static void displayLineSensorDebug(const char *mode_label) {
  static uint32_t debug_cycle_start_ms = 0;
  if (debug_cycle_start_ms == 0) {
    debug_cycle_start_ms = millis();
  }

  char bitmask_buf[17];
  for (int i = 0; i < 16; i++) {
    bitmask_buf[i] = line_sensor_digital[i] ? '1' : '0';
  }
  bitmask_buf[16] = '\0';

  int active_cnt = 0;
  for (int i = 0; i < 16; i++) {
    if (line_sensor_digital[i]) {
      active_cnt++;
    }
  }

  char line3_buf[24];
  if (line_detected) {
    snprintf(line3_buf, sizeof(line3_buf), "POS:%.1f CNT:%d",
             pid_line_position, active_cnt);
  } else {
    snprintf(line3_buf, sizeof(line3_buf), "NO LINE CNT:%d", active_cnt);
  }

  // Rotate through 4 groups of 4 channels every 800ms so raw values for
  // all 16 sensors are visible over time.
  const uint32_t now = millis();
  const uint8_t group = static_cast<uint8_t>((now / 800) % 4);
  const uint8_t ch_start = group * 4;

  char line4_buf[24];
  snprintf(line4_buf, sizeof(line4_buf), "%u:%u %u:%u %u:%u %u:%u", ch_start,
           line_sensor_raw[ch_start], ch_start + 1,
           line_sensor_raw[ch_start + 1], ch_start + 2,
           line_sensor_raw[ch_start + 2], ch_start + 3,
           line_sensor_raw[ch_start + 3]);

  if (!battery_owns_display) {
    displayOLED(mode_label, bitmask_buf, line3_buf, line4_buf);
  }

  static uint32_t last_serial_ms = 0;
  if (now - last_serial_ms >= 200) {
    last_serial_ms = now;
    Serial.printf("[LINE-DEBUG] DIG:%s RAW:", bitmask_buf);
    for (int i = 0; i < 16; i++) {
      Serial.printf(" %d:%u", i, line_sensor_raw[i]);
    }
    Serial.println();
  }
}


// ============== CALIBRATION OUTLIER REJECTION =============
// A plain running min/max (the original approach) has no filtering: a
// single spurious ADC sample -- e.g. an ESP32 ADC2 channel glitching
// during a WiFi TX burst, or a momentary loose connector -- permanently
// poisons that sensor's calibrated range forever, since nothing ever pulls
// max/min back down. In practice this showed up as one sensor's threshold
// getting set far above anything it could realistically read, effectively
// blinding that channel for the whole run.
//
// The fix: any new extreme that represents a big jump from the currently
// accepted max/min (CAL_SPIKE_REJECT_DELTA) is treated as a *candidate*,
// not accepted outright. It only becomes the new accepted max/min once a
// similar value has been seen CAL_CONFIRM_SAMPLES times. Small, gradual
// changes (normal sweeping across the line) are still accepted immediately
// -- only suspiciously large single-sample jumps are held back.
#define CAL_SPIKE_REJECT_DELTA 800
#define CAL_CONFIRM_SAMPLES 3
#define CAL_CONFIRM_TOLERANCE 200

static uint16_t cal_pending_max[16];
static uint8_t cal_pending_max_streak[16];
static uint16_t cal_pending_min[16];
static uint8_t cal_pending_min_streak[16];

static void resetCalibrationCandidates() {
  for (int i = 0; i < 16; i++) {
    cal_pending_max[i] = 0;
    cal_pending_max_streak[i] = 0;
    cal_pending_min[i] = 0;
    cal_pending_min_streak[i] = 0;
  }
}

// Feeds one raw ADC sample into sensor i's running calibration max/min,
// rejecting single-sample spikes until a similar reading repeats.
static void updateCalibrationSample(uint8_t i, uint16_t raw) {
  if (raw > line_sensor_max[i]) {
    if (raw > static_cast<uint16_t>(line_sensor_max[i] + CAL_SPIKE_REJECT_DELTA)) {
      if (cal_pending_max_streak[i] > 0 &&
          abs(static_cast<int>(raw) - static_cast<int>(cal_pending_max[i])) <=
              CAL_CONFIRM_TOLERANCE) {
        cal_pending_max_streak[i]++;
      } else {
        cal_pending_max[i] = raw;
        cal_pending_max_streak[i] = 1;
      }

      if (cal_pending_max_streak[i] >= CAL_CONFIRM_SAMPLES) {
        Serial.printf("[CAL] Sensor %d: confirmed max spike %u (was %u)\n",
                      i, raw, line_sensor_max[i]);
        line_sensor_max[i] = raw;
        cal_pending_max_streak[i] = 0;
      } else {
        Serial.printf("[CAL] Sensor %d: rejected max spike %u (streak %u/%u)\n",
                      i, raw, cal_pending_max_streak[i], CAL_CONFIRM_SAMPLES);
      }
    } else {
      // Small, gradual increase -- trust immediately.
      line_sensor_max[i] = raw;
    }
  }

  if (raw < line_sensor_min[i]) {
    if (static_cast<uint32_t>(raw) + CAL_SPIKE_REJECT_DELTA < line_sensor_min[i]) {
      if (cal_pending_min_streak[i] > 0 &&
          abs(static_cast<int>(raw) - static_cast<int>(cal_pending_min[i])) <=
              CAL_CONFIRM_TOLERANCE) {
        cal_pending_min_streak[i]++;
      } else {
        cal_pending_min[i] = raw;
        cal_pending_min_streak[i] = 1;
      }

      if (cal_pending_min_streak[i] >= CAL_CONFIRM_SAMPLES) {
        Serial.printf("[CAL] Sensor %d: confirmed min dip %u (was %u)\n", i,
                      raw, line_sensor_min[i]);
        line_sensor_min[i] = raw;
        cal_pending_min_streak[i] = 0;
      } else {
        Serial.printf("[CAL] Sensor %d: rejected min dip %u (streak %u/%u)\n",
                      i, raw, cal_pending_min_streak[i], CAL_CONFIRM_SAMPLES);
      }
    } else {
      // Small, gradual decrease -- trust immediately.
      line_sensor_min[i] = raw;
    }
  }
}


// ============== MAIN SEQUENCE =============

void runMainSequence() {
  const uint32_t now = millis();

  checkBatteryAlarm();

  if (!pid_settings_loaded) {
    loadPIDSettings();
  }

  // ---- BTN3: hold >3s to power off ----
  static bool btn3_poweroff_latched = false;
  const bool btn3_down = (button3_last == BUTTON_PRESSED);
  const uint32_t btn3_held_ms = btn3_down ? (now - button3_press_time) : 0;

  if (btn3_down && current_state != STATE_PID_TUNING) {
    if (!btn3_poweroff_latched && btn3_held_ms >= 3000) {
      btn3_poweroff_latched = true;
      displayOLED("POWER OFF", "BTN3 > 3s", "Shutting down", "");
      power(false);
      return;
    }

    char countdown_buf[32];
    const uint32_t remain_ms = (btn3_held_ms >= 3000) ? 0 : (3000 - btn3_held_ms);
    snprintf(countdown_buf, sizeof(countdown_buf), "%.1f s", remain_ms / 1000.0f);
    displayOLED("HOLD BTN3", "Power off in", countdown_buf, "Release=Cancel");
    return;
  }

  btn3_poweroff_latched = false;

  // ----- BTN2: hold >2s to tune PID, or just press to go into idle
  static bool btn2_was_down = false;
  static bool btn2_longpress_fired = false;

  const bool btn2_down = (button2_last == BUTTON_PRESSED);
  const uint32_t btn2_held_ms = btn2_down ? (now - button2_press_time) : 0;

  if (current_state != STATE_PID_TUNING) {
    if (btn2_down) {
      btn2_was_down = true;

      if (!btn2_longpress_fired && btn2_held_ms >= 2000) {
        btn2_longpress_fired = true;
        current_state = STATE_PID_TUNING;
        return;
      }

      char countdown_buf[32];
      const uint32_t remain_ms = (btn2_held_ms >= 2000) ? 0 : (2000 - btn2_held_ms);
      snprintf(countdown_buf, sizeof(countdown_buf), "%.1f s", remain_ms / 1000.0f);
      displayOLED("HOLD BTN2", "OPEN PID TUNING", countdown_buf, "Release=Exit");
      return;
    }

    // button released
    if (btn2_was_down) {
      if (!btn2_longpress_fired) {
        current_state = STATE_IDLE;
        stopMotors();
      }
      btn2_was_down = false;
      btn2_longpress_fired = false;
    }
  }

  // ---- BTN4: tap = toggle line-sensor calibration; hold 2s = line debug ----
  static uint8_t state_before_calibration = STATE_IDLE;
  static bool btn4_was_down = false;
  static bool btn4_longpress_fired = false;

  const bool btn4_down = (button4_last == BUTTON_PRESSED);
  const uint32_t btn4_held_ms = btn4_down ? (now - button4_press_time) : 0;

  if (current_state != STATE_PID_TUNING) {
    if (btn4_down) {
      btn4_was_down = true;

      // Only allow the long-press debug entry when we're not mid-calibration
      // (a hold that started as a calibration toggle shouldn't also jump us
      // into debug mode).
      if (!btn4_longpress_fired && btn4_held_ms >= 2000 &&
          current_state != STATE_CALIBRATING) {
        btn4_longpress_fired = true;
        current_state = STATE_LINE_DEBUG;
        stopMotors();
        Serial.println("[DEBUG] Entering STATE_LINE_DEBUG (BTN4 hold 2s)");
        return;
      }

      // Show a countdown while holding, same pattern as BTN2/BTN3, but only
      // while we're not already inside calibration or debug (those states
      // draw their own screens every frame).
      if (current_state != STATE_CALIBRATING &&
          current_state != STATE_LINE_DEBUG) {
        char countdown_buf[32];
        const uint32_t remain_ms =
            (btn4_held_ms >= 2000) ? 0 : (2000 - btn4_held_ms);
        snprintf(countdown_buf, sizeof(countdown_buf), "%.1f s",
                 remain_ms / 1000.0f);
        displayOLED("HOLD BTN4", "LINE DEBUG in", countdown_buf,
                    "Release=Calibrate");
        return;
      }
    } else {
      // button released
      if (btn4_was_down) {
        if (!btn4_longpress_fired) {
          // Short tap: original calibration start/stop behavior.
          if (current_state != STATE_CALIBRATING) {
            state_before_calibration = current_state;
            current_state = STATE_CALIBRATING;
            is_calibrating = true;
            stopMotors();
            for (int i = 0; i < 16; i++) {
              line_sensor_max[i] = 0;
              line_sensor_min[i] = 4095;
            }
            resetCalibrationCandidates();
            Serial.println("[CAL] Line sensor calibration STARTED (BTN4)");
          } else {
            for (int i = 0; i < 16; i++) {
              line_sensor_threshold[i] = static_cast<uint16_t>(
                  (line_sensor_max[i] + line_sensor_min[i]) / 2);
            }
            // Print raw min/max alongside the derived thresholds so an
            // implausible range (e.g. a max stuck near the 4095 ADC
            // ceiling on a channel that never actually saw the line) is
            // visible immediately, instead of only surfacing later as
            // weird behavior during a run.
            Serial.println("[CAL] Line sensor calibration DONE.");
            for (int i = 0; i < 16; i++) {
              Serial.printf("[CAL]   Sensor %2d: min=%4u max=%4u threshold=%4u\n",
                            i, line_sensor_min[i], line_sensor_max[i],
                            line_sensor_threshold[i]);
            }
            is_calibrating = false;
            current_state = state_before_calibration;
          }
        }
        btn4_was_down = false;
        btn4_longpress_fired = false;
      }
    }
  }

  // ---- BTN1 / BTN2 single-button actions ----
  // Suppressed while the combo is active so a start/stop doesn't fire as
  // a side effect of the two-button hold that's meant for PID tuning.
  const bool btn1_starts = button1_pressed;
  const bool btn2_stops = button2_pressed;

  static uint8_t last_state = STATE_IDLE;

  const bool led_should_be_on = (current_state != STATE_IDLE);
  digitalWrite(LED_POWER_PIN, led_should_be_on ? HIGH : LOW);

  if (last_state != current_state) {
    if (current_state == STATE_PID_TUNING) {
      pid_menu_active = true;
      pid_menu_item = 0;
      pid_enabled = false;
      pid_last_update_ms = 0;
      stopMotors();
      resetPID(&pid_line_following);
    }
    if (current_state == STATE_IDLE) {
      pid_menu_active = false;
      pid_menu_item = 0;
    }

    if (current_state == STATE_PID_FOLLOW) {
      delay(100);
    }

    last_state = current_state;
  }

  // ---- LED_POWER: on whenever actively running (following, turning,
  // calibrating, tuning, debugging); off while idle to save power / avoid
  // glare. ----

  switch (current_state) {

  case STATE_IDLE: {
    syncPIDController();
    displayOLED("IDLE", "BTN1=START", "BTN4=CALIBRATE", "");

    if (btn1_starts) {
      current_state = STATE_PID_FOLLOW;
    }
    break;
  }

  case STATE_PID_FOLLOW: {
    pid_enabled = true;
    followLinePID(pid_base_speed, pid_output_limit);
    break;
  }

  case STATE_CALIBRATING: {
    // Force the illumination LED on (already handled above since this
    // state != STATE_IDLE) so the calibrated range matches the lighting
    // condition the robot will actually run under.
    stopMotors();

    for (int i = 0; i < 16; i++) {
      readLineSensorDigital(i); // refreshes line_sensor_raw[] / _digital[]
      updateCalibrationSample(static_cast<uint8_t>(i), line_sensor_raw[i]);
    }

    char bitmask_buf[17];
    for (int i = 0; i < 16; i++) {
      bitmask_buf[i] = line_sensor_digital[i] ? '1' : '0';
    }
    bitmask_buf[16] = '\0';

    displayOLED("CALIBRATING", bitmask_buf, "Sweep line + bg", "BTN4=Done");
    break;
  }

  case STATE_LINE_DEBUG: {
    // Read-only mode: motors stay off so you can hand-drag the robot over
    // the track and watch the raw values on a laptop Serial Monitor
    // without needing to squint at the tiny OLED while it's moving.
    stopMotors();

    bool should_turn_left = false;
    bool should_turn_right = false;
    const float line_pos =
        calculateLinePosition(should_turn_left, should_turn_right);

    // Throttled to 300ms (was 100ms) and gated so it only prints when the
    // digital bitmask actually changed since the last print -- the robot
    // spends most frames sitting on an unchanged reading, and printing
    // every frame regardless was what made the stream unreadable.
    static uint32_t last_debug_print_ms = 0;
    static char last_dig_buf[17] = "";

    char dig_buf[17];
    for (int i = 0; i < 16; i++) {
      dig_buf[i] = line_sensor_digital[i] ? '1' : '0';
    }
    dig_buf[16] = '\0';

    const bool bitmask_changed = (strcmp(dig_buf, last_dig_buf) != 0);

    if (bitmask_changed || (now - last_debug_print_ms >= 300)) {
      last_debug_print_ms = now;
      strcpy(last_dig_buf, dig_buf);

      Serial.printf("[LINE-DEBUG] t=%lu POS:%.2f DETECTED:%s DIG:%s RAW:",
                    now, line_pos, line_detected ? "YES" : "NO", dig_buf);
      for (int i = 0; i < 16; i++) {
        Serial.printf(" %d:%u/%u", i, line_sensor_raw[i],
                      line_sensor_threshold[i]);
      }
      Serial.printf(" TRIGGER:%s\n",
                    should_turn_left ? "LEFT" :
                    should_turn_right ? "RIGHT" : "-");
    }

    char bitmask_buf[17];
    for (int i = 0; i < 16; i++) {
      bitmask_buf[i] = line_sensor_digital[i] ? '1' : '0';
    }
    bitmask_buf[16] = '\0';

    displayOLED("LINE DEBUG", bitmask_buf, "See Serial Monitor", "BTN2=Exit");
    break;
  }

  case STATE_PID_TUNING: {
    if (button4_pressed) {
      if (!pid_menu_active) {
        pid_menu_active = true;
        pid_menu_item = 0;
      } else if (pid_menu_item < 3) {
        pid_menu_item++;
      } else {
        current_state = STATE_IDLE;
      }
    }

    float kp_step = 0.01f;
    float ki_step = 0.01f;
    float kd_step = 0.01f;
    float base_step = 5.0f;

    const bool btn1_fast =
      (button1_last == BUTTON_PRESSED) &&
      ((now - button1_press_time) >= 1000);

    const bool btn2_fast =
        (button2_last == BUTTON_PRESSED) &&
        ((now - button2_press_time) >= 1000);

    if (btn1_fast || btn2_fast) {
        kp_step = 0.10f;
        ki_step = 0.10f;
        kd_step = 0.10f;
        base_step = 20.0f;
    }

      static uint32_t last_repeat_ms = 0;
      const bool btn1_adjust =
          button1_pressed ||
          (btn1_fast && (now - last_repeat_ms >= 80));

      const bool btn2_adjust =
          button2_pressed ||
          (btn2_fast && (now - last_repeat_ms >= 80));

    if (pid_menu_active && btn1_adjust) {
      switch (pid_menu_item) {
      case 0:
        pid_current_Kp -= kp_step;
        if (pid_current_Kp < 0.0f) {
          pid_current_Kp = 0.0f;
        }
        break;
      case 1:
        pid_current_Ki -= ki_step;
        if (pid_current_Ki < 0.0f) {
          pid_current_Ki = 0.0f;
        }
        break;
      case 2:
        pid_current_Kd -= kd_step;
        if (pid_current_Kd < 0.0f) {
          pid_current_Kd = 0.0f;
        }
        break;
      case 3:
        pid_base_speed -= base_step;
        if (pid_base_speed < 0.0f) {
          pid_base_speed = 0.0f;
        }
        break;
      }
      syncPIDController();
      savePIDSettings();
    }

    if (pid_menu_active && btn2_adjust) {
      switch (pid_menu_item) {
      case 0:
        pid_current_Kp += kp_step;
        if (pid_current_Kp > 50.0f) {
          pid_current_Kp = 50.0f;
        }
        break;
      case 1:
        pid_current_Ki += ki_step;
        if (pid_current_Ki > 10.0f) {
          pid_current_Ki = 10.0f;
        }
        break;
      case 2:
        pid_current_Kd += kd_step;
        if (pid_current_Kd > 50.0f) {
          pid_current_Kd = 50.0f;
        }
        break;
      case 3:
        pid_base_speed += base_step;
        if (pid_base_speed > 255.0f) {
          pid_base_speed = 255.0f;
        }
        break;
      }
      syncPIDController();
      savePIDSettings();
    }

    // ---- BTN3 = one-shot test drive ----
    // A tap (edge-triggered on button3_pressed, not the held level) arms a
    // short window during which followLinePID() actually drives the motors
    // so you can see the correction/response. Outside that window the
    // motors stay off, matching the rest of the tuning menu's behavior.
    static uint32_t pid_test_active_until_ms = 0;

    if (button3_pressed) {                 // fires once per tap, not while held
      pid_test_active_until_ms = now + PID_TEST_DRIVE_MS;
      pid_last_update_ms = 0;              // clean dt, avoid a derivative spike
      resetPID(&pid_line_following);       // clear stale integral from last test
    }

    const bool pid_test_active = (now < pid_test_active_until_ms);

    if (pid_test_active) {
      pid_enabled = true;
      followLinePID(pid_base_speed, pid_output_limit);
    } else if (pid_enabled) {
      // window just ended — stop exactly once, don't spam stopMotors() every frame
      pid_enabled = false;
      pid_last_update_ms = 0;
      stopMotors();
    }

    // Only draw the menu/tuning screen when the test drive isn't running.
    // While pid_test_active is true, followLinePID() -> displayPIDDebug()
    // is the single source of OLED updates for this frame. Drawing both
    // means two full clearDisplay()+display() I2C flushes per loop tick,
    // which is what was causing the flicker / blank-out.
    if (!pid_test_active) {
      char l2[32];
      char l3[32];
      char l4[32];

      if (pid_menu_active) {
        const char *selected = "KP";
        float value = pid_current_Kp;
        if (pid_menu_item == 1) {
          selected = "KI";
          value = pid_current_Ki;
        } else if (pid_menu_item == 2) {
          selected = "KD";
          value = pid_current_Kd;
        } else if (pid_menu_item == 3) {
          selected = "SPD";
          value = pid_base_speed;
        }

        snprintf(l2, sizeof(l2), "SEL:%s VAL:%.3f", selected, value);
        snprintf(l3, sizeof(l3), "BTN1=- BTN2=+");
        snprintf(l4, sizeof(l4), "BTN3=Test BTN4=Next");
        displayOLED("PID MENU", l2, l3, l4);
      } else {
        snprintf(l2, sizeof(l2), "KP:%.2f KI:%.3f", pid_current_Kp,
                 pid_current_Ki);
        snprintf(l3, sizeof(l3), "KD:%.2f SPD:%.0f", pid_current_Kd,
                 pid_base_speed);
        snprintf(l4, sizeof(l4), "BTN4=Menu");
        displayOLED("PID TUNING", l2, l3, l4);
      }
    }
    break;
  }

  default:
    displayOLED("ERROR", "Unknown state", "", "");
    break;
  }

  // BTN2 is the global abort for most running states. Keep it out of
  // PID tuning so BTN2 can be used for parameter adjustment there.
  if (current_state != STATE_IDLE && current_state != STATE_PID_TUNING &&
      btn2_stops) {
    if (current_state == STATE_CALIBRATING) {
      is_calibrating = false;
    }
    current_state = STATE_IDLE;
    stopMotors();
  }
}
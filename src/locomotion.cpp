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

// Existing button polling is implemented in function.cpp.
void pollButtons();

// ---------------------------------------------------------------------------
// NOTE: T_INTERSECTION_SENSOR_CNT / LEFT_CORNER_SENSOR_CNT /
// RIGHT_CORNER_SENSOR_CNT are defined in locomotion.h.
// line_sensor_raw[] / line_sensor_threshold[] are declared `extern` in
// function.h and defined in function.cpp.
//
// *** locomotion.h must gain two new state constants: ***
//   STATE_CALIBRATING
//   STATE_PID_TUNING
// (add them alongside STATE_IDLE / STATE_PID_FOLLOW / etc.)
// ---------------------------------------------------------------------------

uint8_t current_state = STATE_IDLE;

// ========== PID CONTROLLER =============

// PID Controller for line following
PIDController pid_line_following;
bool pid_enabled = true;
float pid_line_position = 0.0f;
bool line_detected = false;
static bool pid_settings_loaded = false;
bool pid_menu_active = false;
static uint8_t pid_menu_item = 0;
uint32_t pid_last_update_ms = 0;
static Preferences pid_preferences;

// PID parameters (adjustable during runtime)
float pid_current_Kp = DEFAULT_KP;
float pid_current_Ki = DEFAULT_KI;
float pid_current_Kd = DEFAULT_KD;
float pid_integral_limit = 100.0f;
float pid_output_limit = DEFAULT_PID_LIMIT;
float pid_base_speed = DEFAULT_PID_BASE_SPEED;

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

bool isLineDetected() { return line_detected; }

void followLinePID(float base_speed, float max_speed_diff) {
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

  const float line_position = calculateLinePosition();
  if (!line_detected) {
    stopMotors();
    resetPID(&pid_line_following);
    return;
  }

  pid_line_following.output_limit = max_speed_diff;
  const float correction =
      calculatePID(&pid_line_following, 7.5f, line_position, dt);

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

  setMotor(1, left_speed);
  setMotor(2, right_speed);
}

// ============== TURNING ===============

#define TURN_PIVOT_SPEED 130   // moderate PWM — tune down if turns overshoot center
#define TURN_CENTER_BAND 1.5f  // |pos - 7.5| within this counts as "centered"
#define TURN_CONFIRM_MS 60u    // must stay centered this long before declaring done
#define TURN_TIMEOUT_MS 1500u  // safety bail-out if the line never reappears

static uint32_t turn_entry_ms = 0;
static uint32_t turn_center_since_ms = 0;

void turnLeft() {
  setMotor(1, -TURN_PIVOT_SPEED);
  setMotor(2, TURN_PIVOT_SPEED);
}

void turnRight() {
  setMotor(1, TURN_PIVOT_SPEED);
  setMotor(2, -TURN_PIVOT_SPEED);
}

void resetTurnTracking(uint32_t now) {
  turn_entry_ms = now;
  turn_center_since_ms = 0;
}

bool turnComplete(uint32_t now) {
  const float pos = calculateLinePosition();

  if (!line_detected) {
    turn_center_since_ms = 0;
  } else if (fabs(pos - 7.5f) <= TURN_CENTER_BAND) {
    if (turn_center_since_ms == 0) {
      turn_center_since_ms = now;
    }
    if (now - turn_center_since_ms >= TURN_CONFIRM_MS) {
      return true; // stably centered -> turn is done
    }
  } else {
    turn_center_since_ms = 0;
  }

  return (now - turn_entry_ms >= TURN_TIMEOUT_MS);
}

// ============== DETECTION =============

uint8_t checkNextState() {
  int active_cnt = 0;
  int active_left_cnt = 0;
  int active_right_cnt = 0;

  float weighted_sum = 0.0f;
  float weight_total = 0.0f;

  for (int i = 0; i < 16; i++) {
    const uint8_t active = readLineSensorDigital(i);
    if (active) {
      active_cnt++;
      const float strength = static_cast<float>(line_sensor_raw[i]) -
                              static_cast<float>(line_sensor_threshold[i]);
      const float weight = (strength > 0.0f) ? strength : 1.0f;
      weighted_sum += static_cast<float>(i) * weight;
      weight_total += weight;

      if (i < 8) {
        active_left_cnt++;
      } else {
        active_right_cnt++;
      }
    }
  }

  if (active_cnt == 0 || weight_total <= 0.0f) {
    return STATE_PID_FOLLOW;
  }

  const float line_pos = weighted_sum / weight_total;

  if (fabs(line_pos - 7.5f) <= 2.0f && active_cnt >= T_INTERSECTION_SENSOR_CNT) {
    return STATE_T_INTERSECTION;
  }

  if (active_left_cnt >= LEFT_CORNER_SENSOR_CNT && active_right_cnt <= 2) {
    return STATE_CORNER_LEFT;
  }

  if (active_right_cnt >= RIGHT_CORNER_SENSOR_CNT && active_left_cnt <= 2) {
    return STATE_CORNER_RIGHT;
  }

  return STATE_PID_FOLLOW;
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

  displayOLED(mode_label, bitmask_buf, line3_buf, line4_buf);

  // Also dump full raw + digital state to Serial periodically for deeper
  // debugging (e.g. via PlatformIO's Serial Monitor).
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

  if (btn3_down) {
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

  // ---- BTN4: toggle line-sensor calibration (press) ----
  static uint8_t state_before_calibration = STATE_IDLE;

  if (button4_pressed && current_state != STATE_PID_TUNING) {
    if (current_state != STATE_CALIBRATING) {
      state_before_calibration = current_state;
      current_state = STATE_CALIBRATING;
      is_calibrating = true;
      stopMotors();
      for (int i = 0; i < 16; i++) {
        line_sensor_max[i] = 0;
        line_sensor_min[i] = 4095;
      }
      Serial.println("[CAL] Line sensor calibration STARTED (BTN4)");
    } else {
      for (int i = 0; i < 16; i++) {
        line_sensor_threshold[i] = static_cast<uint16_t>(
            (line_sensor_max[i] + line_sensor_min[i]) / 2);
      }
      Serial.print("[CAL] Line sensor calibration DONE. Thresholds: ");
      for (int i = 0; i < 16; i++) {
        Serial.printf("%d:%u ", i, line_sensor_threshold[i]);
      }
      Serial.println();
      is_calibrating = false;
      current_state = state_before_calibration;
    }
  }

  // ---- BTN1 / BTN2 single-button actions ----
  // Suppressed while the combo is active so a start/stop doesn't fire as
  // a side effect of the two-button hold that's meant for PID tuning.
  const bool btn1_starts = button1_pressed;
  const bool btn2_stops = button2_pressed;

  static uint8_t last_state = STATE_IDLE;
  if (last_state != current_state) {
    if (current_state == STATE_CORNER_LEFT ||
        current_state == STATE_CORNER_RIGHT) {
      resetTurnTracking(now);
    }
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
    last_state = current_state;
  }

  // ---- LED_POWER: on whenever actively running (following, turning,
  // calibrating, tuning); off while idle to save power / avoid glare. ----
  const bool led_should_be_on = (current_state != STATE_IDLE);
  digitalWrite(LED_POWER_PIN, led_should_be_on ? HIGH : LOW);

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
    // Update sensor readings (this also refreshes line_sensor_digital[]
    // and line_sensor_raw[] used by the debug display below).
    pid_enabled = true;
    followLinePID(pid_base_speed, pid_output_limit);
    displayLineSensorDebug("PID FOLLOW");
    current_state = checkNextState();
    break;
  }

  case STATE_CORNER_LEFT: {
    turnLeft();
    displayLineSensorDebug("CORNER LEFT");
    if (turnComplete(now)) {
      current_state = STATE_PID_FOLLOW;
      resetPID(&pid_line_following);
    }
    break;
  }

  case STATE_CORNER_RIGHT: {
    turnRight();
    displayLineSensorDebug("CORNER RIGHT");
    if (turnComplete(now)) {
      current_state = STATE_PID_FOLLOW;
      resetPID(&pid_line_following);
    }
    break;
  }

  case STATE_T_INTERSECTION: {
    followLinePID(pid_base_speed, pid_output_limit);
    displayLineSensorDebug("T INTERSECT");
    break;
  }

  case STATE_CALIBRATING: {
    // Force the illumination LED on (already handled above since this
    // state != STATE_IDLE) so the calibrated range matches the lighting
    // condition the robot will actually run under.
    stopMotors();

    for (int i = 0; i < 16; i++) {
      readLineSensorDigital(i); // refreshes line_sensor_raw[] / _digital[]
      if (line_sensor_raw[i] > line_sensor_max[i]) {
        line_sensor_max[i] = line_sensor_raw[i];
      }
      if (line_sensor_raw[i] < line_sensor_min[i]) {
        line_sensor_min[i] = line_sensor_raw[i];
      }
    }

    char bitmask_buf[17];
    for (int i = 0; i < 16; i++) {
      bitmask_buf[i] = line_sensor_digital[i] ? '1' : '0';
    }
    bitmask_buf[16] = '\0';

    displayOLED("CALIBRATING", bitmask_buf, "Sweep line + bg", "BTN4=Done");
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
        pid_menu_active = false;
        current_state = STATE_IDLE;
      }
    }

    const float kp_step = 0.01f;
    const float ki_step = 0.01f;
    const float kd_step = 0.01f;
    const float base_step = 5.0f;

    if (pid_menu_active && button1_pressed) {
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

    if (pid_menu_active && button2_pressed) {
      switch (pid_menu_item) {
      case 0:
        pid_current_Kp += kp_step;
        if (pid_current_Kp > 10.0f) {
          pid_current_Kp = 10.0f;
        }
        break;
      case 1:
        pid_current_Ki += ki_step;
        if (pid_current_Ki > 1.0f) {
          pid_current_Ki = 1.0f;
        }
        break;
      case 2:
        pid_current_Kd += kd_step;
        if (pid_current_Kd > 10.0f) {
          pid_current_Kd = 10.0f;
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

    if (pid_enabled) {
      followLinePID(pid_base_speed, pid_output_limit);
    } else {
      stopMotors();
      calculateLinePosition();
    }

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
      snprintf(l4, sizeof(l4), "BTN4=Next BTN3=%s", pid_enabled ? "STOP" : "RUN");
      displayOLED("PID MENU", l2, l3, l4);
    } else {
      snprintf(l2, sizeof(l2), "KP:%.2f KI:%.3f", pid_current_Kp,
               pid_current_Ki);
      snprintf(l3, sizeof(l3), "KD:%.2f SPD:%.0f", pid_current_Kd,
               pid_base_speed);
      snprintf(l4, sizeof(l4), "BTN4=Menu BTN3=%s", pid_enabled ? "STOP" : "RUN");
      displayOLED("PID TUNING", l2, l3, l4);
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
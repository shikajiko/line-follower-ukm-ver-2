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

uint8_t current_state = STATE_IDLE

// ========== PID CONTROLLER =============

// PID Controller for line following
PIDController pid_line_following;
bool pid_enabled = true;
float pid_line_position = 0.0f;
bool line_detected = false;
static bool pid_settings_loaded = false;
static bool pid_menu_active = false;
static uint8_t pid_menu_item = 0;
static uint32_t pid_last_update_ms = 0;
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

static void loadPIDSettings() {
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

// ============== DETECTION =============
uint8_t checkNextState() {
    int active_cnt = 0;
    bool is_active[16] = {false};
    for (int i = 0; i < 16; i++) {
        const uint8_t active = readLineSensorDigital(i);
        if (active) {
            active_cnt++;
            is_active[i] = true;
            const float strength = static_cast<float>(line_sensor_raw[i]) -
                                    static_cast<float>(line_sensor_threshold[i]);
            const float weight = (strength > 0.0f) ? strength : 1.0f;
            weighted_sum += static_cast<float>(i) * weight;
            weight_total += weight;
        }
    }

    line_pos = weighted_sum / weight_total;

    if (fabs(line_pos - 7.5) <= 2.0 && active_cnt >= T_INTERSECTION_SENSOR_CNT) 
        return STATE_T_INTERSECTION;
    
    int corner_active = 0;
    for (int i = 0; i < 8; i++) {
        if (is_active[i]) corner_active++;
    }

    if (corner_active > LEFT_CORNER_SENSOR_CNT && active_cnt < 10) 
        return STATE_CORNER_LEFT;

    corner_active = 0;
    for (int i = 8; i < 16; i++) {
        if (is_active[i]) corner_active++;
    }

    if (corner_active > RIGHT_CORNER_SENSOR_CNT && active_cnt < 10) 
        return STATE_CORNER_RIGHT;

    return STATE_PID_FOLLOW;
}

void runMainSequence() {
    uint32_t now = millis();

    checkBatteryAlarm();

    switch (current_state) {
    
    case STATE_IDLE:
        syncPIDController();
        displayOLED("IDLE", "BTN 1", "START", "");

        int btn1_state = digitalRead(BUTTON1_PIN);
        if (btn1_state == BUTTON_PRESSED) {
            current_state = STATE_PID_FOLLOW;
        }

        break;

    case STATE_PID_FOLLOW:
        followLinePID(pid_base_speed, pid_output_limit);
        break;

    case STATE_CORNER_LEFT:
        break;
    case STATE_CORNER_RIGHT:
        break;
    case STATE_T_INTERSECTION: 

    }

    current_state = checkNextState();
}
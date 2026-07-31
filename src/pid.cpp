#include "pid.h"
#include "stdint.h"
#include "display.h"
#include "line_sensor.h"
#include <Preferences.h>

PIDController pid_line_following;
bool pid_enabled = true;
float pid_line_position = 0.0f;
float prev_pid_correction = 0.0f;
uint32_t pid_last_update_ms = 0;

static bool pid_settings_loaded = false;
static Preferences pid_preferences;

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
  int left_weight = 0;
  int right_weight = 0;

  float weighted_sum = 0.0f;
  float weight_total = 0.0f;

  for (int i = 0; i < 16; i++) {
    const uint8_t active = getLineSensorDigital(i);
    if (active) {
      const float strength = static_cast<float>(getLineSensorRaw(i)) -
                              static_cast<float>(getLineSensorThreshold(i));
      const float weight = (strength > 0.0f) ? strength : 1.0f;
      weighted_sum += static_cast<float>(i) * weight;
      weight_total += weight;
    }
  }

  if (weight_total <= 0.0f) {
    return pid_line_position;
  }

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
    bitmask_buf[i] = getLineSensorDigital(i) ? '1' : '0';
  }
  bitmask_buf[16] = '\0';

  int active_cnt = 0;
  for (int i = 0; i < 16; i++) {
    if (getLineSensorDigital(i)) {
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
  
  displayOLED(bitmask_buf, line2_buf, line3_buf, "");
}

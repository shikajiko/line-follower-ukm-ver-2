#include "pid.h"
#include "stdint.h"
#include "display.h"
#include "line_sensor.h"
#include <Preferences.h>
#include <array>

static PIDController pid;
bool pid_enabled = true;
float pid_line_position = 0.0f;

static bool pid_settings_loaded = false;
static Preferences pid_preferences;

void initPIDController() {
  pid.Kp = DEFAULT_KP;
  pid.Ki = DEFAULT_KI;
  pid.Kd = DEFAULT_KD;
  pid.integral = 0.0f;
  pid.last_error = 0.0f;
  pid.integral_limit = 0;
  pid.output_limit = DEFAULT_PID_LIMIT;
  pid.base_speed = DEFAULT_PID_BASE_SPEED;
}

std::array<float, 3> getPIDCoefficient() {
    return {pid.Kp, pid.Ki, pid.Kd};
}

float getPIDBaseSpeed() {
    return pid.base_speed;
}

void enablePID() {
    pid_enabled = true;
}

void disablePID() {
    pid_enabled = false;
}

bool isPIDEnabled() {
    return pid_enabled;
}

bool isPIDSettingsLoaded() {
    return pid_settings_loaded;
}

float calculatePID(float setpoint, float current_value,
                    float dt) {
  const float error = setpoint - current_value;
  pid.integral += error * dt;

  if (pid.integral > pid.integral_limit) {
    pid.integral = pid.integral_limit;
  } else if (pid.integral < -pid.integral_limit) {
    pid.integral = -pid.integral_limit;
  }

  const float derivative = (error - pid.last_error) / dt;
  pid.last_error = error;

  float output =
      (pid.Kp * error) + (pid.Ki * pid.integral) + (pid.Kd * derivative);

  if (output > pid.output_limit) {
    output = pid.output_limit;
  } else if (output < -pid.output_limit) {
    output = -pid.output_limit;
  }

  return output;
}

void resetPID() {
  pid.integral = 0.0f;
  pid.last_error = 0.0f;
}

void savePIDSettings(float new_kp, float new_ki, float new_kd, float new_base_speed) {
  pid_preferences.begin("pidline", false);
  pid_preferences.putFloat("kp", new_kp);
  pid_preferences.putFloat("ki", new_ki);
  pid_preferences.putFloat("kd", new_kd);
  pid_preferences.putFloat("base", new_base_speed);
  pid_preferences.end();

  pid.Kp = new_kp;
  pid.Ki = new_ki;
  pid.Kd = new_kd;
  pid.base_speed = new_base_speed;
}

void loadPIDSettings() {
  if (pid_settings_loaded) {
    return;
  }

  ensureLineSensorThresholdDefaults();

  pid_preferences.begin("pidline", true);
  pid.Kp = pid_preferences.getFloat("kp", DEFAULT_KP);
  pid.Ki = pid_preferences.getFloat("ki", DEFAULT_KI);
  pid.Kd = pid_preferences.getFloat("kd", DEFAULT_KD);
  pid.base_speed = pid_preferences.getFloat("base", DEFAULT_PID_BASE_SPEED);
  pid_preferences.end();

  if (pid.Kp < 0.0f) {
    pid.Kp = 0.0f;
  }
  if (pid.Ki < 0.0f) {
    pid.Ki = 0.0f;
  }
  if (pid.Kd < 0.0f) {
    pid.Kd = 0.0f;
  }
  if (pid.base_speed < 0.0f) {
    pid.base_speed = 0.0f;
  } else if (pid.base_speed > 255.0f) {
    pid.base_speed = 255.0f;
  }

  resetPID();
  pid_settings_loaded = true;
}

void resetPIDValues() {
  pid.Kp = DEFAULT_KP;
  pid.Ki = DEFAULT_KI;
  pid.Kd = DEFAULT_KD;
  pid.base_speed = DEFAULT_PID_BASE_SPEED;

  resetPID();
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

void displayPIDDebug(const float line_pos, const float correction, int16_t right_speed, int16_t left_speed, bool is_line_detected) {
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

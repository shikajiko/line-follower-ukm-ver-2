#include "locomotion.h"
#include "pid.h"
#include "IO.h"
#include "line_sensor.h"
#include "display.h"
#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <driver/pcnt.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

// PID Controller for line following
bool line_detected = false;
float prev_pid_correction = 0.0f;
static uint32_t line_lost_ms;
static uint32_t pid_last_update_ms = 0;

void moveMotors(int16_t left_speed, int16_t right_speed) {
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

void stopMotors() {
  ledcWrite(MOTOR1_IN1_CH, 0);
  ledcWrite(MOTOR1_IN2_CH, 0);
  ledcWrite(MOTOR2_IN3_CH, 0);
  ledcWrite(MOTOR2_IN4_CH, 0);
  digitalWrite(INH1_PIN, LOW);

  motor1_speed = 0;
  motor2_speed = 0;
}

void followLinePID() {
  if (!isPIDEnabled()) {
    stopMotors();
    return;
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
  line_detected = isLineDetected();

  float correction = calculatePID(DEFAULT_PID_SETPOINT, line_position, dt);

  if (!line_detected) {
    if (line_lost_ms == 0) {
      line_lost_ms = now;
    }
    if (now - line_lost_ms > 1000) {
      stopMotors();
      resetPID();
      line_lost_ms = 0;
      return;

    }
    correction = prev_pid_correction;
  } else {
    line_lost_ms = 0;
    correction =
      calculatePID(DEFAULT_PID_SETPOINT, line_position, dt);
      prev_pid_correction = correction;
  }

  int16_t left_speed = static_cast<int16_t>(getPIDBaseSpeed() - correction);
  int16_t right_speed = static_cast<int16_t>(getPIDBaseSpeed() + correction);

  moveMotors(left_speed, right_speed);
  displayPIDDebug(line_position, correction, right_speed, left_speed, line_detected);
}

// static void displayLineSensorDebug(const char *mode_label) {
//   static uint32_t debug_cycle_start_ms = 0;
//   if (debug_cycle_start_ms == 0) {
//     debug_cycle_start_ms = millis();
//   }

//   char bitmask_buf[17];
//   for (int i = 0; i < 16; i++) {
//     bitmask_buf[i] = line_sensor_digital[i] ? '1' : '0';
//   }
//   bitmask_buf[16] = '\0';

//   int active_cnt = 0;
//   for (int i = 0; i < 16; i++) {
//     if (line_sensor_digital[i]) {
//       active_cnt++;
//     }
//   }

//   char line3_buf[24];
//   if (line_detected) {
//     snprintf(line3_buf, sizeof(line3_buf), "POS:%.1f CNT:%d",
//              pid_line_position, active_cnt);
//   } else {
//     snprintf(line3_buf, sizeof(line3_buf), "NO LINE CNT:%d", active_cnt);
//   }

//   // Rotate through 4 groups of 4 channels every 800ms so raw values for
//   // all 16 sensors are visible over time.
//   const uint32_t now = millis();
//   const uint8_t group = static_cast<uint8_t>((now / 800) % 4);
//   const uint8_t ch_start = group * 4;

//   char line4_buf[24];
//   snprintf(line4_buf, sizeof(line4_buf), "%u:%u %u:%u %u:%u %u:%u", ch_start,
//            line_sensor_raw[ch_start], ch_start + 1,
//            line_sensor_raw[ch_start + 1], ch_start + 2,
//            line_sensor_raw[ch_start + 2], ch_start + 3,
//            line_sensor_raw[ch_start + 3]);

//   if (!battery_owns_display) {
//     displayOLED(mode_label, bitmask_buf, line3_buf, line4_buf);
//   }

//   static uint32_t last_serial_ms = 0;
//   if (now - last_serial_ms >= 200) {
//     last_serial_ms = now;
//     Serial.printf("[LINE-DEBUG] DIG:%s RAW:", bitmask_buf);
//     for (int i = 0; i < 16; i++) {
//       Serial.printf(" %d:%u", i, line_sensor_raw[i]);
//     }
//     Serial.println();
//   }
// }


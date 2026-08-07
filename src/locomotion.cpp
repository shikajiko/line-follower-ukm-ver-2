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

  if (REVERSE_RIGHT_MOTOR) {
    right_speed *= -1;
  }

  if (REVERSE_MOTOR) {
    int temp = left_speed;
    left_speed = right_speed;
    right_speed = temp;
  }

  setMotor(1, left_speed);
  setMotor(2, right_speed);
}

void stopMotors() {
  moveMotors(0, 0);
}

void followLinePID(bool prioritize_straight, bool is_inverted, int16_t baseSpeed) {
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

  const float line_position = calculateLinePosition(prioritize_straight, is_inverted);

  line_detected = isLineDetected();

  float correction;

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
    correction = calculatePID(0.0f, line_position, dt);
    prev_pid_correction = correction;
  }

  int16_t left_speed = static_cast<int16_t>(baseSpeed - correction);
  int16_t right_speed = static_cast<int16_t>(baseSpeed + correction);

  moveMotors(left_speed, right_speed);
  displayPIDDebug(line_position, correction, right_speed, left_speed, true);
}

void brakeMotors()
{
    moveMotors(-255, -255);
    moveMotors(0, 0);
}
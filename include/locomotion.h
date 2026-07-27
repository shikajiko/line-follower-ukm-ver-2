#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdint.h>
#include "function.h"

#define STATE_IDLE 0
#define STATE_PID_FOLLOW 1
#define STATE_CORNER_RIGHT 2
#define STATE_CORNER_LEFT 3
#define STATE_T_INTERSECTION 4

#define LEFT_CORNER_SENSOR_CNT 6
#define RIGHT_CORNER_SENSOR_CNT 6
#define T_INTERSECTION_SENSOR_CNT 8

// ==== MAIN STATE MACHINE =====
void runMainSequence();
extern uint8_t current_state;

// ==== LOCOMOTION FUNCTION =====
void turnRight();
void turnLeft();
uint8_t checkNextState();

// ==== PID ====
void initPIDController(PIDController *pid, float Kp, float Ki, float Kd,
                        float integral_limit, float output_limit);
float calculatePID(PIDController *pid, float setpoint, float current_value,
                    float dt);
void resetPID(PIDController *pid);

float calculateLinePosition();
void followLinePID(float base_speed, float max_speed_diff);
// void setPIDTuning(float Kp, float Ki, float Kd);

// loadPIDSettings() is idempotent (guards itself with an internal flag),
// so it's exposed here — function.cpp's runTestSequence() calls it every
// frame without needing to see the guard flag itself.
void loadPIDSettings();
void resetPIDValues();

// PID runtime state, defined in locomotion.cpp, needed by function.cpp's
// test/debug screens (printPIDDebug, runTestSequence).
extern PIDController pid_line_following;
extern bool pid_enabled;
extern bool pid_menu_active;
extern uint32_t pid_last_update_ms;
extern float pid_line_position;
extern bool line_detected;
extern float pid_current_Kp;
extern float pid_current_Ki;
extern float pid_current_Kd;
extern float pid_integral_limit;
extern float pid_output_limit;
extern float pid_base_speed;

#endif // locomotion.h
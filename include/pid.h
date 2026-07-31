#ifndef PID_H
#define PID_H

#define DEFAULT_KP 0.2f
#define DEFAULT_KI 0.f
#define DEFAULT_KD 1.f
#define DEFAULT_PID_LIMIT 120.f
#define DEFAULT_PID_BASE_SPEED 140.f
#define DEFAULT_PID_SETPOINT 8.0f

typedef struct {
  float Kp;              // Proportional gain
  float Ki;              // Integral gain
  float Kd;              // Derivative gain
  float integral;        // Integral accumulator
  float last_error;      // Previous error for derivative calculation
  float integral_limit;  // Anti-windup limit for integral term
  float output_limit;    // Maximum output value
} PIDController;

void initPIDController(PIDController *pid, float Kp, float Ki, float Kd,
                        float integral_limit, float output_limit);

float calculatePID(PIDController *pid, float setpoint, float current_value,
                    float dt);

void resetPID(PIDController *pid);
void resetPIDValues();
void loadPIDSettings();
void displayPIDDebug();
float calculateLinePosition();

#endif PID_H //pid.h
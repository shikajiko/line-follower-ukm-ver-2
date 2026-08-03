#ifndef PID_H
#define PID_H

#define DEFAULT_KP 0.2f
#define DEFAULT_KI 0.f
#define DEFAULT_KD 1.f
#define DEFAULT_PID_LIMIT 120.f
#define DEFAULT_PID_BASE_SPEED 140.f

#include <array>
#include <stdint.h>

typedef struct {
  float Kp;              // Proportional gain
  float Ki;              // Integral gain
  float Kd;              // Derivative gain
  float integral;        // Integral accumulator
  float last_error;      // Previous error for derivative calculation
  float integral_limit;  // Anti-windup limit for integral term
  float output_limit;    // Maximum output value
  float base_speed;       // Base speed
} PIDController;

void initPIDController();

float calculatePID(float setpoint, float current_value,
                    float dt);

void enablePID();
void disablePID();
bool isPIDEnabled();

bool isPIDSettingsLoaded();
void resetPID();
void resetPIDValues();
void loadPIDSettings();
void savePIDSettings(float new_kp, float new_ki, float new_kd, float new_base_speed);

std::array<float, 3> getPIDCoefficient();
float getPIDBaseSpeed();

void displayPIDDebug(const float line_pos, const float correction, int16_t right_speed, int16_t left_speed, bool is_line_detected);
float calculateLinePosition();


#endif //pid.h
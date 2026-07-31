#ifndef FUNCTION_H
#define FUNCTION_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdint.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define LIMIT_BATTERY 11.1f
// Global OLED display object
extern Adafruit_SSD1306 display;

// Default PID values
#define DEFAULT_KP 0.2f
#define DEFAULT_KI 0.f
#define DEFAULT_KD 1.f
#define DEFAULT_PID_LIMIT 120.f
#define DEFAULT_PID_BASE_SPEED 140.f
#define DEFAULT_PID_SETPOINT 8.0f

// Line sensor
#define LINE_SENSOR_THRESHOLD 3600

// ============ INIT FUNCTIONS ============

void initOLED();
void initBuzzer();
void initButton();
void initEncoder();
void initMotor();
void initADC();
void initMUX();
void initGY25();
void initServo();
void setupPCNT();
void initpower();
void power(bool state);

// ============ TEST STATE MACHINE ============

void runTestSequence();
void runSingleSensorCheck();

// ============ TEST MODULE FUNCTIONS ============

// GY25 IMU
int16_t readGY25Yaw();

// Encoder PCNT
int32_t readEncoder(uint8_t encoderNum);
void resetEncoder(uint8_t encoderNum);

// Servo control (PWM1, PWM2)
void setServo(uint8_t servoNum, uint16_t pulseUs);

// Motor control (IN + shared enable)
void setMotor(uint8_t motorNum, int16_t speed); // -255 to +255
void stopMotors();

// Line sensor 16 channels via MUX+ADC
uint16_t readLineSensor(uint8_t channel); // raw ADC value
// Menggunakan threshold internal yang bisa dikalibrasi
uint8_t readLineSensorDigital(uint8_t channel); // binary ON/OFF

// Utility display/serial
void displayOLED(const char *line1, const char *line2, const char *line3,
                 const char *line4);
void printSerial(const char *msg);
void pollButtons();

float readBatteryVoltage();
void checkBatteryAlarm();

// ============ PID CONTROL SYSTEM ============

// PID controller structure
typedef struct {
  float Kp;              // Proportional gain
  float Ki;              // Integral gain
  float Kd;              // Derivative gain
  float integral;        // Integral accumulator
  float last_error;      // Previous error for derivative calculation
  float integral_limit;  // Anti-windup limit for integral term
  float output_limit;    // Maximum output value
} PIDController;

void calibrateLineSensorsAuto();
bool isLineDetected();
void ensureLineSensorThresholdDefaults();

// PID debug and monitoring
void printPIDDebug();
void resetPIDValues();

// ============ GLOBAL STATE ============
// These are DECLARATIONS ONLY (no initializer). The initializer is what
// turns `extern int x = 0;` into a definition — putting that in a header
// included by multiple .cpp files causes "multiple definition" linker
// errors. The actual definitions live in function.cpp.

extern int16_t gy25_yaw;   // x100 degrees
extern int16_t gy25_pitch; // x100 degrees
extern int16_t gy25_roll;  // x100 degrees

// Motor state
extern int16_t motor1_speed;
extern int16_t motor2_speed;

// Line sensor
extern uint16_t line_sensor_raw[16];
extern uint8_t line_sensor_digital[16];
extern bool is_calibrating;

// Line sensor calibration arrays
extern uint16_t line_sensor_max[16];
extern uint16_t line_sensor_min[16];
extern uint16_t line_sensor_threshold[16];

// Battery low display
extern bool battery_owns_display;

#endif
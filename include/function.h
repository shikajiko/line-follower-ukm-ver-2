#ifndef FUNCTION_H
#define FUNCTION_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdint.h>

#define LIMIT_BATTERY 11.1f
// Global OLED display object
extern Adafruit_SSD1306 display;


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

// Utility display/serial
void printSerial(const char *msg);
void pollButtons();

float readBatteryVoltage();
void checkBatteryAlarm();

// PID debug and monitoring
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


// Battery low display
extern bool battery_owns_display;

#endif
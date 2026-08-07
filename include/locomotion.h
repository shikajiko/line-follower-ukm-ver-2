#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdint.h>
#include "function.h"

#define MOTOR1_IN1_CH 2
#define MOTOR1_IN2_CH 3
#define MOTOR2_IN3_CH 4
#define MOTOR2_IN4_CH 5

void followLinePID(bool prioritize_straight, bool is_inverted, int16_t baseSpeed);
void moveMotors(int16_t left_speed, int16_t right_speed);
void stopMotors();
void brakeMotors();

#endif // locomotion.h
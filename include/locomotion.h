#ifndef LOCOMOTION_H
#define LOCOMOTION_H

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdint.h>
#include "function.h"

#define STATE_IDLE 0
#define STATE_PID_FOLLOW 1
#define STATE_TURN_LEFT 2
#define STATE_TURN_RIGHT 3


// ==== MAIN STATE MACHINE =====
void runMainSequence();

// ==== LOCOMOTION FUNCTION =====
void turnRight();
void turnLeft();

// ==== TRACK UTILITIES =====
bool isTJunction();
bool shouldTurnLeft();
bool shouldTurnRight(); 

// ==== UTILITIES ==== 
float estimateTurnDeg();

#endif // locomotion.h
#ifndef MAIN_SEQUENCE_H
#define MAIN_SEQUENCE_H

#include "stdint.h"

#define STATE_IDLE 0
#define STATE_PID_FOLLOW 1
#define STATE_CORNER_RIGHT 2
#define STATE_CORNER_LEFT 3
#define STATE_T_INTERSECTION 4
#define STATE_CALIBRATING 5
#define STATE_PID_TUNING 6
#define STATE_LINE_DEBUG 7

void runMainSequence();

#endif //main_sequence.h
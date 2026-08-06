#ifndef MAIN_SEQUENCE_H
#define MAIN_SEQUENCE_H

#include "stdint.h"

#define STATE_IDLE 0
#define STATE_RUN_MISSION 1
#define STATE_CALIBRATING 2
#define STATE_PID_TUNING 3
#define STATE_LINE_DEBUG 4
#define STATE_WEB_SERVER 5

void runStateMachine();

#endif //main_sequence.h
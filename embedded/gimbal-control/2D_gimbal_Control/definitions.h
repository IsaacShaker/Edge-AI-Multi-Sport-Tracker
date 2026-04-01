#include <Arduino.h>

#ifndef MY_HEADER_H // Header guard
#define MY_HEADER_H

/**
MOTOR SPECIFICATIONS
*/

#define POLE_PAIRS 7
#define PHASE_RESISTANCE 2.3f
#define KV_RATING 220
#define Q_INDUCTANCE 0.00086f


/**
MOTOR PINTOUTS
*/

#define B_PH1 1
#define B_PH2 2
#define B_PH3 3
#define B_EN  5

#define T_PH1 6
#define T_PH2 7
#define T_PH3 8
#define T_EN  5

/**
POWER TELEMETRY PINOUTS
*/

#define V_12 
#define V_5
#define V_3
#define VB_PH1
#define VB_PH2
#define VB_PH3
#define VT_PH1
#define VT_PH2
#define VT_PH3
#define GAIN

#endif // MY_HEADER_H




#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

//robot dimension stuff
static const float CIRC = 0.25132741228; // meters
static const float BASE = 0.08; // meters
static const float METERS_PER_TICK = 0.00020944; // meters per tick of encoder

//maebot driving stuff
const static float GO = 0.4;
const static float STOP = 0.0;
const static float TURN_SPEED_SCALE = 0.5;
const static float TURN_ANGLE_SCALE = 0.65;
const static float KP = 1.0;
const static float KD = 1.95;

//directions
const static float RIGHT = 0.0;
const static float LEFT = M_PI;
const static float UP = M_PI / 2.0;
const static float DOWN = -M_PI / 2.0;


#endif

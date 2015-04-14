#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

namespace eecs467{

//robot dimension stuff
static const float CIRC = 0.25132741228; // meters
static const float BASE = 0.08; // meters
static const float METERS_PER_TICK = 0.00020944; // meters per tick of encoder

//maebot driving stuff
const static float GO = 0.3;
const static float RIGHT_OFFSET_1 = 0.9;
const static float STOP = 0.0;
const static float TURN_SPEED_SCALE = 0.7;
const static float TURN_ANGLE_SCALE = 0.9;
const static float KP = 1.5;
const static float KI = 0.0;
const static float KD = 0.0;
const static int SENSOR_OFFSET_1 = -30;

//directions
const static float RIGHT = 0.0;
const static float LEFT = M_PI;
const static float UP = M_PI / 2.0;
const static float DOWN = -M_PI / 2.0;

//Game driver
const static float END_PROG = 123.456;

}

#endif

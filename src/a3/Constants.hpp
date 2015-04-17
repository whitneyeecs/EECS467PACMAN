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
const static float TURN_SPEED_SCALE = 0.72;
const static float TURN_ANGLE_SCALE = 0.8;
const static float KP = 0.38;
const static float KI = 0.0;
const static float KD = 1.151515151515151515151515151515;
const static int SENSOR_OFFSET_1 = -30;

//directions
const static float RIGHT = 0.0;
const static float LEFT = M_PI;
const static float UP = M_PI / 2.0;
const static float DOWN = -M_PI / 2.0;

//Game driver
const static float END_PROG = 123.456;

//stuff for initializing nav
const static int PACMAN = 1; // bot 1 is set up to be pacman
const static int DUMBGHOST = 8; //bot 8 is set up to be dumbghost
const static int SMARTGHOST = -1;

//board
const static float boardCellSize = 0.1; // meters

const static float TURN_THRESHOLD = M_PI / 6;
const static float TARGET_RADIUS = 0.07;

}

#endif

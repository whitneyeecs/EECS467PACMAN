#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"

#include "math/angle_functions.hpp"
#include "math/point.hpp"
#include <cmath>

//a3
#include "node.h"

class Navigation {

private:

	

public:

	Navigation();

	/*
	*/
	void turning();

	/*
	correction of following lines using infared sensors
	*/
	void lineFollowing();

	/*
	smart ghost path planning to pacman's next location(node)
	returns next node to go to
	*/
	Node pathPlanning();

	/*
	dumb ghost path, returns next node to go to
	*/
	Node randomPath();

	/*
	returns the pacmans next location
	*/
	Node pacmanLoc();


};

#endif

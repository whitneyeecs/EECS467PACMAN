#ifndef MAP_HPP
#define MAP_HPP

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"

#include "Constants.hpp"

#include <iostream>
#include <vector>

using namespace std;

class Map {

private:

public:

	struct Node {

		maebot_pose_t pose;

		int nodeNum;
		int right;
		int left;
		int front;
		int behind;
	};

	vector <Node> map;

	Map();

	/*
	returns a vector that represents the map, easy version
	*/
	vector <Map::Node> initilizeEasy();

	/*
	returns a vector that represents the map, normal pacman version
	*/
	vector <Map::Node> initilizeNorm();

	/*
	initilizes map based on requested type of map
	*/
	void getMap(char mapType);

};

#endif

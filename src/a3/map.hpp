#ifndef MAP_HPP
#define MAP_HPP

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"

#include "Constants.hpp"

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

	Map();


	/*
	returns a vector that represents the map, easy version
	*/
	vector <Map::Node> initilizeEasy();

	/*
	returns a vector that represents the map, medium version
	*/
	vector <Map::Node> initilizeMed();

	/*
	returns a vector that represents the map, hard version
	*/
	vector <Map::Node> initilizeHard();

};

#endif

#ifndef MAP_HPP
#define MAP_HPP

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"

#include "Constants.hpp"

#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

class Map {

private:

public: 

	struct Node {

		int nodeNum;
		int xPos;
		int yPos;
		int up;
		int down;
		int left;
		int right;
	};

	vector <Node> map;

	Map();
 
	/*
	returns a vector that represents the map
	*/
	vector <Map::Node> initilize();

	/*
	initilizes map 
	*/
	void getMap();

}; 

#endif

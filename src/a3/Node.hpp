#ifndef NODE_HPP
#define NODE_HPP

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"

class Node {

public:

	maebot_pose_t pose;

	bool forward;
	bool backward;
	bool right;
	bool left;
	
	Node();

	Node::Node(maebot_pose_t p, bool f, bool b, bool l, bool r);

};

#endif

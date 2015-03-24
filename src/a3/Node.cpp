#include "Node.hpp"

Node::Node() { }

Node::Node(maebot_pose_t p, bool f, bool b, bool l, bool r){

	pose = p;
	forward = f;
	backward = b;
	right = r;
	left = l;
}



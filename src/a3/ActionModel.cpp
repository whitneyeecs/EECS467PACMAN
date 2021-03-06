#include "a3/ActionModel.hpp"
#include "a3/StateEstimator.hpp"
#include "a3/RobotConstants.hpp"

#include <cmath>
#include <chrono>

#include "math/angle_functions.hpp"

eecs467::ActionModel::ActionModel(float k1, float k2) 
	: _k1(k1), _k2(k2) {
	_rand = gslu_rand_rng_alloc();		
}

void eecs467::ActionModel::apply(maebot_pose_t& pose, int32_t deltaRight, int32_t deltaLeft, int64_t utime) {
	maebot_pose_t nextPose = eecs467::advanceState(pose,
		deltaRight, deltaLeft, utime);

	float deltaX = nextPose.x - pose.x;
	float deltaY = nextPose.y - pose.y;
	float deltaTheta = nextPose.theta - pose.theta;

	float deltaS = sqrt(deltaX * deltaX + deltaY * deltaY);
	float alpha = wrap_to_pi(atan2(deltaY, deltaX) - pose.theta);

	// getting noise
	float e1 = gslu_rand_normal(_rand) * alpha * _k1;
	float e2 = gslu_rand_normal(_rand) * deltaS * _k2;
	float e3 = gslu_rand_normal(_rand) * wrap_to_pi(deltaTheta - alpha) * _k1;

	pose.x += (deltaS + e2)* cos(pose.theta + alpha + e1);
	pose.y += (deltaS + e2)* sin(pose.theta + alpha + e1);
	pose.theta += wrap_to_pi(deltaTheta + e1 + e3);
	pose.utime = nextPose.utime;
}


void eecs467::ActionModel::apply(maebot_pose_t& pose, int32_t deltaRight, int32_t deltaLeft, float deltaS, int64_t utime) {
	maebot_pose_t nextPose = eecs467::advanceState(pose,
		deltaRight, deltaLeft, utime);

	float deltaX = nextPose.x - pose.x;
	float deltaY = nextPose.y - pose.y;
	float deltaTheta = nextPose.theta - pose.theta;

	float alpha = wrap_to_pi(atan2(deltaY, deltaX) - pose.theta);

	// getting noise
	float e1 = gslu_rand_normal(_rand) * alpha * _k1;
	float e2 = gslu_rand_normal(_rand) * deltaS * _k2;
	float e3 = gslu_rand_normal(_rand) * wrap_to_pi(deltaTheta - alpha) * _k1;

	pose.x += (deltaS + e2)* cos(pose.theta + alpha + e1);
	pose.y += (deltaS + e2)* sin(pose.theta + alpha + e1);
	pose.theta += wrap_to_pi(deltaTheta + e1 + e3);
	pose.utime = nextPose.utime;
}

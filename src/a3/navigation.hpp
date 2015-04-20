#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <iostream>
#include <unistd.h>

#include "Constants.hpp"

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/maebot_motor_feedback_t.hpp>
#include <lcmtypes/maebot_motor_command_t.hpp>
#include <lcmtypes/maebot_sensor_data_t.hpp>
#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <lcmtypes/maebot_pose_t.hpp>

#include "common/timestamp.h"

#include "math/angle_functions.hpp"
#include "math/point.hpp"
#include <cmath>

namespace eecs467{

class Navigation{
private:	
	//command to be sent in loop and updated in 
	//motion functions
	maebot_motor_command_t cmd;

	//feedback to be used for line following and 
	//turning
	maebot_motor_feedback_t odo;

	//is have we recieved first odometry?
	bool odo_init;

	//pose to be used for localization and control
	maebot_pose_t pose;

	//line sensors on bottom of MAE-Bot
	int32_t sensors[3];

	//instance of lcm for handling feedback
	//and commands within class
	lcm::LCM lcm;

	//a mutex
	pthread_mutex_t mutex;

	//robot's pose: X Y THETA
//	float pose[3];

	//set true when driving forward, false when stopped
	bool driving;

	//some things for line following control
	int32_t error;
	int32_t prev_error;

	//who are you?
	int player;

	float RIGHT_OFFSET;

	Point<float> cur_dest;
public:
	Navigation(int player);

	void test();

	//call this function to publish motor command messages
	void publish();

	//this function sets the motor command 'cmd' values
	//to GO. if a turn is required this function performs
	//it before forward motion. 
	//Sets driving to true
	void go(float dir);

	//this function will hopefully take you from where you are to the 
	//point that you input
	void go_to(Point<float> dest);

	//sets motor command 'cmd' values to STOP.
	//sets driving to false.
	void stop();
	
	//calls lcm.handle(). calls all lcm handlers.
	void handle();

	//this function returns true if bot is driving
	//can be used to check when to update autonomous path finding
	bool is_driving() { return driving; } 

	//pushes pose from lidar localization to correct the running
	//odometry pose estimate
	void push_pose(maebot_pose_t laser_pose);

	maebot_pose_t get_pose() {
		pthread_mutex_lock(&mutex);
		maebot_pose_t temp = pose;
		pthread_mutex_unlock(&mutex);
		return temp;
	}

private:
	//pushes motor feedback to odo
	void handle_feedback(const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const maebot_motor_feedback_t* msg);
						        
	//stores values of line sensor in sensors[]							
	
	//turns robot by 'angle' which should always be a multiple
	//of +- pi/2. stores 'end_dir' into pose[], this is the 
	//direction that the bot ends up facing
	void turn(float angle, float end_dir);

	//PD controller keeps maebot on line using sensors[]
	void correct(Point<float> dest);

}; // end class

}// end namespace

#endif

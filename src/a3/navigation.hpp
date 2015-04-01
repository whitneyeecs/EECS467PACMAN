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

#include "common/timestamp.h"

class Navigation{
private:	
	//command to be sent in loop and updated in 
	//motion functions
	maebot_motor_command_t cmd;

	//feedback to be used for line following and 
	//turning
	maebot_motor_feedback_t odo;

	//line sensors on bottom of MAE-Bot
	int32_t sensors[3];

	//instance of lcm for handling feedback
	//and commands within class
	lcm::LCM lcm;

	//a mutex
	pthread_mutex_t mutex;

	//robot's pose: X Y THETA
	float pose[3];

	//set true when driving forward, false when stopped
	bool driving;

	//some things for line following control
	int32_t error;
	int32_t prev_error;
public:
	Navigation();

	//call this function to publish motor command messages
	void publish();

	//this function sets the motor command 'cmd' values
	//to GO. if a turn is required this function performs
	//it before forward motion. 
	//Sets driving to true
	void go(float dir);

	//sets motor command 'cmd' values to STOP.
	//sets driving to false.
	void stop();
	
	//calls lcm.handle(). calls all lcm handlers.
	void handle();

private:
	//pushes motor feedback to odo
	void handle_feedback(const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const maebot_motor_feedback_t* msg);
						        
	//stores values of line sensor in sensors[]							
	void handle_sensor(const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const maebot_sensor_data_t* msg);

	
	void handle_laser(const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const maebot_laser_scan_t* msg);
	
	//turns robot by 'angle' which should always be a multiple
	//of +- pi/2. stores 'end_dir' into pose[], this is the 
	//direction that the bot ends up facing
	void turn(float angle, float end_dir);

	//PD controller keeps maebot on line using sensors[]
	void correct();

}; // end class


#endif

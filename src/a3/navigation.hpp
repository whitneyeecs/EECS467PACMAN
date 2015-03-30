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
	maebot_motor_command_t cmd;

	maebot_motor_feedback_t odo;

	int32_t sensors[3];

	lcm::LCM lcm;

	pthread_mutex_t mutex;

	float pose[3];

	bool driving;
public:
	Navigation();

	void publish();

	void go(float dir);

	void stop();
	
	void handle();

private:
	void handle_feedback(const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const maebot_motor_feedback_t* msg);
						        
	void handle_sensor(const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const maebot_sensor_data_t* msg);

	void handle_laser(const lcm::ReceiveBuffer* rbuf,
							const std::string& chan,
							const maebot_laser_scan_t* msg);
	
	void turn(float angle, float end_dir);

	void correct();

}; // end class


#endif

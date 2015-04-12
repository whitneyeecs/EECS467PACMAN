#include "navigation.hpp"
#include <iostream>

Navigation::Navigation():
	sensors {0, 0, 0},
	pose{0.0, 0.0, RIGHT},
	driving(false),
	error(0),
	prev_error(0)
	{
		if(!lcm.good()){
			std::cout << "error initializing LCM !!!" << std::endl;
			exit(1);
		}

		if(pthread_mutex_init(&mutex, NULL)){
			std::cout << "error initializing mutex !!!" << std::endl;
			exit(1);
		}

		lcm.subscribe("MAEBOT_MOTOR_FEEDBACK", 
						&Navigation::handle_feedback, this);
		lcm.subscribe("MAEBOT_SENSOR_DATA", 
						&Navigation::handle_sensor, this);
		lcm.subscribe("MAEBOT_LASER_SCAN",
						&Navigation::handle_laser, this);
		
	}





void Navigation::handle_feedback(const lcm::ReceiveBuffer* rbuf,
                     const std::string& chan,
                     const maebot_motor_feedback_t* msg){
	pthread_mutex_lock(&mutex);
/*	int64_t old_time = odo.utime;
	float hz = 1000000.0 / (float)(msg->utime - old_time);
	std::cout << hz << std::endl;
*/	odo = *msg;
	pthread_mutex_unlock(&mutex);
	return;
}






                               
void Navigation::handle_sensor(const lcm::ReceiveBuffer* rbuf,
                   const std::string& chan,
                   const maebot_sensor_data_t* msg){
	pthread_mutex_lock(&mutex);
	for(int i = 0; i < 3; ++i){
		sensors[i] = msg->line_sensors[i];
	}
	pthread_mutex_unlock(&mutex);

	return;
}







void Navigation::handle_laser(const lcm::ReceiveBuffer* rbuf,
                  const std::string& chan,
                  const maebot_laser_scan_t* msg){
	return;
}







void Navigation::handle(){
	lcm.handle();
}







void Navigation::publish(){
	pthread_mutex_lock(&mutex);
	lcm.publish("MAEBOT_MOTOR_COMMAND", &cmd);
	pthread_mutex_unlock(&mutex);

	return;
}







void Navigation::correct(){
	int32_t cur_error = 0;
	int32_t prev_error = 0;
	int32_t int_error = 0;
	float correct = 0.0;


	int hz = 50;
	while(driving){
		cur_error = sensors[2] - sensors[0];// - SENSOR_OFFSET_1;
printf("error:\t%d\n", cur_error);
		int_error += cur_error;
		correct = KP * (float)cur_error + KI * (float)int_error + 
			  KD * (float)(cur_error - prev_error);
		prev_error = cur_error;

		
		
		if(correct > 0.0){
			pthread_mutex_lock(&mutex);
			cmd.motor_left_speed = GO + 0.1 * (correct / 200.0);
			cmd.motor_right_speed = GO * RIGHT_OFFSET_1;// - 0.1 * (correct / 200.0);
			cmd.utime = utime_now();
			pthread_mutex_unlock(&mutex);
		}else{
			pthread_mutex_lock(&mutex);
			cmd.motor_left_speed = GO;// + 0.1 * (correct / 200.0);
			cmd.motor_right_speed = GO * RIGHT_OFFSET_1 - 0.1 * (correct / 200.0);
			cmd.utime = utime_now();
			pthread_mutex_unlock(&mutex);
		}
			usleep(1000000 / hz);
	}
}







void Navigation::go(float dir){
	float cur_dir = 0.0;
	pthread_mutex_lock(&mutex);
	cur_dir = pose[2];
	pthread_mutex_unlock(&mutex);
	if(dir != cur_dir){
		turn(dir - cur_dir, dir);	
	}
	if(!driving){
		pthread_mutex_lock(&mutex);
		cmd.motor_left_speed = GO;
		cmd.motor_right_speed = GO * RIGHT_OFFSET_1;
		cmd.utime = utime_now();
		driving = true;
		pthread_mutex_unlock(&mutex);
//	}else{
		correct();
	}
}







void Navigation::stop(){
	pthread_mutex_lock(&mutex);
	cmd.motor_left_speed = STOP;
	cmd.motor_right_speed = STOP;
	cmd.utime = utime_now();
	driving = false;
	pthread_mutex_unlock(&mutex);
}







void Navigation::turn(float angle, float end_dir){
	
	stop();
	
	int32_t left_start = 0,right_start = 0,
			left_cur = 0, right_cur = 0;

	pthread_mutex_lock(&mutex);
	left_start = left_cur = odo.encoder_left_ticks;
	right_start = right_cur = odo.encoder_right_ticks;
	pthread_mutex_unlock(&mutex);
	float arc_len = CIRC * (angle / (2 * M_PI));
	int32_t delta_ticks = arc_len / METERS_PER_TICK;
	delta_ticks = abs(delta_ticks) * TURN_ANGLE_SCALE;


	int sign = 0;

	if(angle < 0.0){
		sign = -1;
	}else{
		sign = 1;
	}
std::cout << "arc lenght: " << arc_len << "\ndelta ticks: " << delta_ticks <<std::endl;	

	pthread_mutex_lock(&mutex);
	cmd.motor_left_speed = GO * TURN_SPEED_SCALE * -sign;
	cmd.motor_right_speed = GO * RIGHT_OFFSET_1 * TURN_SPEED_SCALE * sign;
	cmd.utime = utime_now();
	pthread_mutex_unlock(&mutex);
	
	int hz = 1000;
	while(abs(right_cur - right_start) < delta_ticks ||
		abs(left_cur - left_start) < delta_ticks){
		
		pthread_mutex_lock(&mutex);
		left_cur = odo.encoder_left_ticks;
		right_cur = odo.encoder_right_ticks;
		pthread_mutex_unlock(&mutex);
		
		if(abs(right_cur - right_start) >= delta_ticks){
			pthread_mutex_lock(&mutex);
			cmd.motor_right_speed = STOP;
			cmd.utime = utime_now();
			pthread_mutex_unlock(&mutex);
			
		}
		if(abs(left_cur - left_start) >= delta_ticks){
			pthread_mutex_lock(&mutex);
			cmd.motor_left_speed = STOP;
			cmd.utime = utime_now();
			pthread_mutex_unlock(&mutex);
			
		}


		usleep(1000000 / hz);
	}

	

	stop();

	pthread_mutex_lock(&mutex);
	pose[2] = end_dir;
	pthread_mutex_unlock(&mutex);
	
}






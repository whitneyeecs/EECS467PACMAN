#include "navigation.hpp"
#include <iostream>
#include <math.h>
#include "common/timestamp.h"

namespace eecs467{

Navigation::Navigation(int player):
	odo_init(false),
	sensors {0, 0, 0},
	driving(false),
	error(0),
	prev_error(0),
	player(0)
	{	

		if(!lcm.good()){
			std::cout << "error initializing LCM !!!" << std::endl;
			exit(1);
		}

		if(pthread_mutex_init(&mutex, NULL)){
			std::cout << "error initializing mutex !!!" << std::endl;
			exit(1);
		}

		if(player == PACMAN){
			lcm.subscribe("PACMAN_MOTOR_FEEDBACK", 
						&Navigation::handle_feedback, this);
			pose.x = 0.0;
			pose.y = 0.0;
			pose.theta = 0.0;
			RIGHT_OFFSET = RIGHT_OFFSET_1;

		}else if(player == SMARTGHOST){
			lcm.subscribe("SMARTGHOST_MOTOR_FEEDBACK", 
						&Navigation::handle_feedback, this);
			pose.x = 0.0;//pose.x = 304.8; 
			pose.y = 0.0;
			pose.theta = M_PI;
			RIGHT_OFFSET = 1.0;
		}
	}






void Navigation::handle_feedback(const lcm::ReceiveBuffer* rbuf,
                     const std::string& chan,
                     const maebot_motor_feedback_t* msg){
	pthread_mutex_lock(&mutex);
	if(odo_init){
		int delta_l = msg->encoder_left_ticks - odo.encoder_left_ticks;
		int delta_r = msg->encoder_right_ticks - odo.encoder_right_ticks;

		float delta_dist = METERS_PER_TICK * (float)(delta_l + delta_r) / 2.0;
		float delta_theta = METERS_PER_TICK * (float)(delta_r - delta_l) / BASE;

		pose.x += delta_dist * cos(pose.theta + delta_theta);
		pose.y += delta_dist * sin(pose.theta + delta_theta);
		pose.theta += delta_theta;
		pose.utime = msg->utime;
	}
	odo = *msg;
	odo_init = true;
	pthread_mutex_unlock(&mutex);
	return;
}





void Navigation::push_pose(maebot_pose_t laser_pose){
	if(!driving){
		pthread_mutex_lock(&mutex);
		pose = laser_pose;
		pose.utime = utime_now();
		pthread_mutex_unlock(&mutex);
	}
}






void Navigation::handle(){
	lcm.handle();
}







void Navigation::publish(){
	pthread_mutex_lock(&mutex);
	lcm.publish("PACMAN_MOTOR_COMMAND", &cmd);
	pthread_mutex_unlock(&mutex);
	return;
}







void Navigation::correct(Point<float> dest){
//printf("entering Correct\n");
	float cur_error = 0.0;
	float prev_error = -1000.0;
	float int_error = 0.0;
	float correct = 0.0;

	int hz = 60;
	while(driving){
		Point<float> cur;
		float cur_theta;
		pthread_mutex_lock(&mutex);
		cur.x = pose.x;
		cur.y = pose.y;
		cur_theta = pose.theta;
		pthread_mutex_unlock(&mutex);

		float dist_to_target = distance_between_points(cur, dest);
		if(dist_to_target <= eecs467::TARGET_RADIUS){
			stop();
			//break;
			return;			
		}

		
		
		float target_angle = atan2(dest.y - cur.y, dest.x - cur.x);
		cur_error = wrap_to_pi(target_angle - cur_theta);

		if(abs(cur_error) > eecs467::TURN_THRESHOLD){
			turn(cur_error, 0.0);
			cur_error = 0.0;
			prev_error = -1000.0;
			int_error = 0.0;
			usleep(3000000);
		}else{ 
			if(prev_error == -1000.0) prev_error = cur_error;
			int_error += cur_error;
			correct = KP * (float)cur_error + KI * (float)int_error + 
				  KD * (float)(cur_error - prev_error);
			prev_error = cur_error;
	
			
			if(correct > 0.0){
				pthread_mutex_lock(&mutex);
				cmd.motor_left_speed = GO;// + 0.1 * (correct / 200.0);
				cmd.motor_right_speed = GO * RIGHT_OFFSET + (correct / M_PI);
				cmd.utime = utime_now();
				pthread_mutex_unlock(&mutex);
			}else{
				pthread_mutex_lock(&mutex);
				cmd.motor_left_speed = GO - (correct / M_PI);
				cmd.motor_right_speed = GO * RIGHT_OFFSET;// - 0.1 * (correct / 200.0);
				cmd.utime = utime_now();
				pthread_mutex_unlock(&mutex);
			}
				usleep(1000000 / hz);
		}
	}

}





void Navigation::go_to(Point<float> dest){
printf("entering go_to\n");	
	pthread_mutex_lock(&mutex);
	if(driving){
		pthread_mutex_unlock(&mutex);
		return;
	}
	driving = true;
	pthread_mutex_unlock(&mutex);
	
	correct(dest);
}






void Navigation::go(float dir){
	float cur_dir = 0.0;
	pthread_mutex_lock(&mutex);
	cur_dir = pose.theta;//pose[2];
	pthread_mutex_unlock(&mutex);
	if(abs(dir - cur_dir) > M_PI / 4.0){
		turn(dir - cur_dir, dir);	
	}
	if(!driving){
		pthread_mutex_lock(&mutex);
		cmd.motor_left_speed = GO;
		cmd.motor_right_speed = GO * RIGHT_OFFSET;
		cmd.utime = utime_now();
		driving = true;
		pthread_mutex_unlock(&mutex);
//		correct();
	}
}







void Navigation::stop(){
	pthread_mutex_lock(&mutex);
	cmd.motor_left_speed = STOP;
	cmd.motor_right_speed = STOP;
	cmd.utime = utime_now();
	driving = false;
//std::cout << pose.x << " " << pose.y << " " << pose.theta << std::endl;
	pthread_mutex_unlock(&mutex);
}







void Navigation::turn(float angle, float end_dir){
	
//	stop();
	
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

	int hz = 1000;
	pthread_mutex_lock(&mutex);
	cmd.motor_left_speed = GO * TURN_SPEED_SCALE * -sign;
	cmd.motor_right_speed = GO * RIGHT_OFFSET * TURN_SPEED_SCALE * sign;
	cmd.utime = utime_now();
	pthread_mutex_unlock(&mutex);
	
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
	
}

void Navigation::test(){
	while(1){
	pthread_mutex_lock(&mutex);
	cmd.motor_left_speed = GO;
	cmd.motor_right_speed = GO * (1.0);
	cmd.utime = utime_now();
	pthread_mutex_unlock(&mutex);
	}
}


}// end namespace



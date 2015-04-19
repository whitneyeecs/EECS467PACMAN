#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>

#include "common/zarray.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

//lcm
#include "lcm/lcm-cpp.hpp"
#include "lcmtypes/pacman_command_t.hpp"
#include <lcmtypes/maebot_occupancy_grid_t.hpp>
#include <lcmtypes/maebot_motor_feedback_t.hpp>
#include <lcmtypes/maebot_laser_scan_t.hpp>
#include <lcmtypes/maebot_pose_t.hpp>
#include <lcmtypes/maebot_map_data_t.hpp>
#include <lcmtypes/maebot_sensor_data_t.hpp>
#include "mapping/occupancy_grid.hpp"
//#include "lcmtypes/maebot_board_locations_t.hpp"

#include "eecs467_util.h"    // This is where a lot of the internals live

#include "math/point.hpp"

#include "a3/navigation.hpp"
#include "a3/board.hpp"

using namespace eecs467;

typedef struct state state_t;
struct state {
 
   	bool running;


	image_u32_t image;

	Navigation* nav;
	Board* board;

    	// 	threads
    	pthread_t odo_thread;
	pthread_t command_thread;
	pthread_t test_thread;
	
	

    // for accessing the arrays
    pthread_mutex_t mutex;

	
	Point<int> cur_board_pos;
	Point<int> next_board_pos;

	lcm::LCM* lcm;


	void handle_pose(const lcm::ReceiveBuffer* rbuf, 
				const std::string& chan,
				const maebot_pose_t* msg){

		nav->push_pose(*msg);
	}

	void handle_command(const lcm::ReceiveBuffer* rbuf,
				const std::string& chan, 
				const pacman_command_t* msg){
		if(msg->command == 1.0){
			nav->stop();
		}
		else if(msg->command == END_PROG){
			running = false;
		}
		else{
//			nav->go(msg->command);
printf("pacman got a command at: %d\n", msg->utime);
			pthread_mutex_lock(&mutex);
			next_board_pos = board->nextWaypoint(cur_board_pos, msg->command);
			pthread_mutex_unlock(&mutex);
		}
	}

};

void* test_thread(void* arg){
	state_t* state = (state_t*) arg;
/*	Point<float> one (0.0, 2.0);
	Point<float> two (1.0, 2.0);
	Point<float> three (1.0, 0.0);
	Point<float> four (0.0, 0.0);

	usleep(5000000);	
	state->nav->go_to(one);
	while(state->nav->is_driving()){};
	state->nav->go_to(two);
	while(state->nav->is_driving()){};
	state->nav->go_to(three);
	while(state->nav->is_driving()){};
	state->nav->go_to(four);
*/
	Point<float> dest ( 0.0, 0.0);
	Point<float> cur (0.0, 0.0);
	maebot_pose_t pose;

	int hz = 50;
	while(1){
		pthread_mutex_lock(&state->mutex);
		if(state->next_board_pos.x != -1 && !state->nav->is_driving() 
				&& state->cur_board_pos != state->next_board_pos){
			dest = state->board->convertToGlobalCoords(state->next_board_pos);

printf("goint to\nX:\t%d\tY:\t%d\n", state->next_board_pos.x, state->next_board_pos.y);			
			pthread_mutex_unlock(&state->mutex);
			state->nav->go_to(dest);

		}else{
			pthread_mutex_unlock(&state->mutex);
		}
		
		pose = state->nav->get_pose();
		
		pthread_mutex_lock(&state->mutex);
		state->cur_board_pos = state->board->convertToBoardCoords(pose);
		pthread_mutex_unlock(&state->mutex);
		
		usleep(1000000 / hz);
	}

	return NULL;
	
}

void* command_thread(void* arg){
	int hz = 50;
	state_t* state = (state_t*) arg;
	while(state->running){
		state->nav->publish();
		usleep(1000000 / hz);
	}
	return NULL;
}

void *
odo_thread (void *data)
{
    state_t *state = (state_t*)data;

	while(state->running){
		state->nav->handle();
	}

    return NULL;
}

state_t *
state_create (void)
{
   	state_t *state = (state_t*)calloc (1, sizeof(*state));

   	state->running = 1;

	state->cur_board_pos.x = 0;
	state->cur_board_pos.y = 0;

	state->next_board_pos.x = 0;
	state->next_board_pos.y = 0;
	
	state->nav = new Navigation(PACMAN);
	state->board = new Board();
   	
	state->lcm = new lcm::LCM; 
    	if(!state->lcm->good()){
		std::cout <<"error initializing LCM !!!" << std::endl;
		exit(1);
	}

	state->lcm->subscribe("PACMAN_COMMAND", &state::handle_command, state);
    
	state->lcm->subscribe("PACMAN_POSE", &state::handle_pose, state);	
    
    	return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;

	delete state->nav;
	delete state->lcm;

    free (state);
}

int
main (int argc, char *argv[])
{
//	eecs467_init (argc, argv);
	state_t *state = state_create ();

    


	// Launch our worker threads
	pthread_create (&state->odo_thread, NULL, odo_thread, state);
	pthread_create (&state->command_thread, NULL, command_thread, state);
	pthread_create (&state->test_thread, NULL, test_thread, state);

	while(state->running){
		state->lcm->handle();
	}

	pthread_join (state->odo_thread, NULL);
	pthread_join (state->command_thread, NULL);
	pthread_join (state->test_thread, NULL);

	// Cleanup
	state_destroy (state);


}















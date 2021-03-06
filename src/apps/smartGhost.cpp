#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <stack>

#include "common/zarray.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

//lcm
#include "lcm/lcm-cpp.hpp"
#include <lcmtypes/maebot_pose_t.hpp>

#include "eecs467_util.h"    // This is where a lot of the internals live

#include "math/point.hpp"

#include "a3/navigation.hpp"
#include "a3/board.hpp"
#include "a3/Constants.hpp"

using namespace eecs467;

typedef struct state state_t;
struct state {
 
   	bool running;

	image_u32_t image;

	//game stuff
	Navigation* nav;
	Board* board;

	stack < Point<int> > wayPoints; 

	//threads
	pthread_t odo_thread;
	pthread_t command_thread;
	pthread_t ai_thread;

    pthread_mutex_t mutex;

	lcm::LCM* lcm;

	maebot_pose_t pose;
	Point <int> pacmanDest;

	void pose_handler(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
					   const maebot_pose_t* msg)
	{
		pose = *msg;
		nav->push_pose(*msg);
	}

	void pacmanDest_handler(const lcm::ReceiveBuffer* rbuf, 
								const std::string& chan, const Point <int> *msg)
	{
		pacmanDest = *msg;
	}
	
};

void* command_thread(void* arg){
	int hz = 50;
	state_t* state = (state_t*) arg;
	while(state->running){
		state->nav->publish();
		usleep(1000000 / hz);
	}
	return NULL;
}

void * odo_thread (void *data)
{
    state_t *state = (state_t*)data;

	while(state->running){
		state->nav->handle();
	}

    return NULL;
}

void * ai_thread(void *data)
{
	state_t *state = (state_t*)data;

	//THE CHASE!!!
	while(!state->nav->is_driving()){

		Point <int> myLocation = state->board->convertToBoardCoords(state->pose);	
		
		state->wayPoints = state->board->getPath(myLocation, state->pacmanDest);
		
		if(!state->wayPoints.empty()) {	
			Point <int> point;
			point.x = state->wayPoints.top().x;
			point.y = state->wayPoints.top().y;
			state->wayPoints.pop();
			Point <float> dest = state->board->convertToGlobalCoords(point); 
			state->nav->go_to(dest);
		}	
	}

	return NULL;
}

state_t *
state_create (void)
{
   	state_t *state = (state_t*)calloc (1, sizeof(*state));

   	state->running = 1;
	state->nav = new Navigation(SMARTGHOST);
   	state->board = new Board();

	state->pose.x = 3.048;
	state->pose.y = 0.0;
	state->pose.theta = M_PI;

	state->lcm = new lcm::LCM; 
    	if(!state->lcm->good()){
		std::cout <<"error initializing LCM !!!" << std::endl;
		exit(1);
	}
    	
	state->lcm->subscribe("SMARTGHOST_POSE", &state::pose_handler, state);
    //state->lcm->subscribe("PACMAN_DEST", &state::pacmanDest_handler, state);

    return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;

	delete state->nav;
	delete state->board;
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
	pthread_create (&state->ai_thread, NULL, ai_thread, state);

	state->pose.x = 304.8;
	state->pose.y = 0.0;
	state->pose.theta = M_PI;

	while(state->running){
		state->lcm->handle();
	}

	pthread_join (state->odo_thread, NULL);
	pthread_join (state->command_thread, NULL);
	pthread_join (state->ai_thread, NULL);

	// Cleanup
	state_destroy (state);


}

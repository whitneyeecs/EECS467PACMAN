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
 
	int diffy; // larger the number the easier, the smaller the harder
	int diffyCount;

	stack < Point<int> > wayPoints; 
	
	Point <int> myLocation;
    
	//threads
    	pthread_t odo_thread;
	pthread_t command_thread;
	pthread_t ai_thread;

    	// for accessing the arrays
    	pthread_mutex_t mutex;

	lcm::LCM* lcm;

	maebot_pose_t pose;

	void pose_handler(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
					   const maebot_pose_t* msg)
	{
		pose = *msg;
		nav->push_pose(*msg);
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
	
	//RANDOM EXPLORE
	while(!state->nav->is_driving()){
	
		Point <int> myLocation = state->board->convertToBoardCoords(state->pose);	
		
		if(state->diffyCount > state->diffy || state->wayPoints.empty()) {
			//get new path
			Point <int> dest;
			dest.x = rand() % state->board->width;
			dest.y = rand() % state->board->height;
			state->wayPoints = state->board->getPath(myLocation, dest);			
			state->diffyCount = 0;
		}
		
		else {
			Point <float> point;
			point.x = state->wayPoints.top().x;
			point.y = state->wayPoints.top().y;
			state->wayPoints.pop();
			Point <float> dest = state->board->convertToGlobalCoords(point);
			state->nav->go_to(dest);			
			state->diffyCount++;
		}	
	}
	
	return NULL;
}

state_t *
state_create (void)
{
   	state_t *state = (state_t*)calloc (1, sizeof(*state));

   	state->running = 1;
	state->nav = new Navigation(DUMBGHOST);
	state->board = new Board;
	state->diffyCount = 0;
	state->diffy = 2;

	state->pose.x = 0.0;
	state->pose.y = 304.8;
	state->pose.theta = 0.0;

	state->lcm = new lcm::LCM; 
    	if(!state->lcm->good()){
		std::cout <<"error initializing LCM !!!" << std::endl;
		exit(1);
	}

	state->lcm->subscribe("DUMBGHOST_POSE", &state::pose_handler, state);    

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

	state->pose.x = 0.0;
	state->pose.y = 3.048;
	state->pose.theta = 0.0;

	while(state->running){
		//state->lcm->handle();
		state->pose = state->nav->get_pose()
	}

	pthread_join (state->odo_thread, NULL);
	pthread_join (state->command_thread, NULL);
	pthread_join (state->ai_thread, NULL);

	// Cleanup
	state_destroy (state);


}

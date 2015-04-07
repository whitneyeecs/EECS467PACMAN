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
#include "lcmtypes/maebot_board_locations_t.hpp"

#include "eecs467_util.h"    // This is where a lot of the internals live

#include "a3/navigation.hpp"
#include "a3/map.hpp"

typedef struct state state_t;
struct state {
 
   	bool running;


	image_u32_t image;

	Navigation* nav;


    // threads
    pthread_t odo_thread;
	pthread_t command_thread;

	//map
	Map map;

    // for accessing the arrays
    pthread_mutex_t mutex;


	lcm::LCM* lcm;

	void handle_command(const lcm::ReceiveBuffer* rbuf,
				const std::string& chan, 
				const pacman_command_t* msg){
		if(msg->command == 1.0){
			nav->stop();
		}
		else if(msg->command == END_PROG){
			std::cout <<"Should End" << std::endl;
			running = false;
		}
		else{
			std::cout << "GOT A MESSAGE" << std::endl;
			nav->go(msg->command);
		}
	}
};


void* command_thread(void* arg){
	int hz = 20;
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
	state->nav = new Navigation;
   	
	state->lcm = new lcm::LCM; 
    	if(!state->lcm->good()){
		std::cout <<"error initializing LCM !!!" << std::endl;
		exit(1);
	}

	state->lcm->subscribe("PACMAN_COMMAND", &state::handle_command, state);
    
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

	while(state->running){
		state->lcm->handle();
	}

	pthread_join (state->odo_thread, NULL);
	pthread_join (state->command_thread, NULL);

	// Cleanup
	state_destroy (state);


}















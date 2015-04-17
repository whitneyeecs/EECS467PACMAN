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
//#include "a3/LaserCorrector.hpp"
//#include "a3/ParticleFilter.hpp"
//#include "a3/Mapper.hpp"
//#include "a3/SlamConstants.hpp"
//#include "a3/RobotConstants.hpp"
//#include "a3/map.hpp"

using namespace eecs467;

typedef struct state state_t;
struct state {
 
   	bool running;


	image_u32_t image;

	Navigation* nav;


    	// 	threads
    	pthread_t odo_thread;
	pthread_t command_thread;
	pthread_t test_thread;
	
	//map
//	Map map;

    // for accessing the arrays
    pthread_mutex_t mutex;


	lcm::LCM* lcm;
//        eecs467::LaserCorrector* laser;
//        eecs467::ParticleFilter* pf;
//        eecs467::Mapper* mapper;


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
			nav->go(msg->command);
		}
	}
/*
	void handle_laser(const lcm::ReceiveBuffer* rbuf,
                        const std::string& chan,
                        const maebot_laser_scan_t* msg){

                pthread_mutex_lock(&mutex);
                if(!pf->processing()){
                        pf->pushScan(*msg);
                }
//printf("got a laser\n");
                pthread_mutex_unlock(&mutex);
        }

        void handle_feedback(const lcm::ReceiveBuffer* rbuf,
                        const std::string& chan,
                        const maebot_motor_feedback_t* msg){

                pthread_mutex_lock(&mutex);
                pf->pushOdometry(*msg);

                if(pf->readyToInit() && !pf->initialized()){
                        pf->init(msg);
                        printf("initialized particle filter\n");
                }

                if(pf->readyToProcess() && pf->initialized()){
                        //get current pose
                        maebot_pose_t oldPose = pf->getBestPose();

                        //process pf
                        pf->process();

                        // get pose after a move
                        maebot_pose_t newPose = pf->getBestPose();
                        //broadcast new pose here
			pac_pose = newPose;
			nav->push_pose(newPose);

                        //get corrected laser scans
                        maebot_processed_laser_scan_t processedScans =
                                laser->processSingleScan(*pf->getScan(), oldPose, newPose);

                        //update map
                        mapper->update(processedScans);

                        //broadcast map
                        maebot_particle_map_t pfmap;
                        pf->toLCM(pfmap);
                        lcm->publish("MAEBOT_PARTICLE_MAP", &pfmap);
                }
                pthread_mutex_unlock(&mutex);
        }
*/

};
/*
void* test_thread(void* arg){
	state_t* state = (state_t*) arg;
	Point<float> one (0.0, 1.0);
	Point<float> two (0.0, 0.0);

	usleep(5000000);	
	state->nav->go_to(one);
	while(state->nav->is_driving()){};
	state->nav->go_to(two);

	return NULL;
	
}
*/
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
	state->nav = new Navigation(PACMAN);
   	
	state->lcm = new lcm::LCM; 
    	if(!state->lcm->good()){
		std::cout <<"error initializing LCM !!!" << std::endl;
		exit(1);
	}

	state->lcm->subscribe("PACMAN_COMMAND", &state::handle_command, state);
    
	state->lcm->subscribe("PACMAN_POSE", &state::handle_pose, state);	
/*	state->lcm->subscribe("PACMAN_LASER_SCAN",
                        &state::handle_laser, state);

    	state->lcm->subscribe("PACMAN_MOTOR_FEEDBACK",
                        &state::handle_feedback, state);

    	state->laser = new LaserCorrector;
    	state->pf = new ParticleFilter;
    	state->mapper = new Mapper(eecs467::gridSeparationSize,
                eecs467::gridWidthMeters,
                eecs467::gridHeightMeters,
                eecs467::gridCellSizeMeters);

    	state->pf->pushMap(state->mapper->getGrid());
*/
    
    	return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;

	delete state->nav;
	delete state->lcm;
//    	delete state->laser;
 //   	delete state->pf;
  //  	delete state->mapper;



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
//	pthread_create (&state->test_thread, NULL, test_thread, state);

	while(state->running){
		state->lcm->handle();
	}

	pthread_join (state->odo_thread, NULL);
	pthread_join (state->command_thread, NULL);

	// Cleanup
	state_destroy (state);


}















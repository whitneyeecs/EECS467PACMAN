#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

//LCM
#include "lcm/lcm-cpp.hpp"
#include "lcmtypes/pacman_command_t.hpp"

#include "eecs467_util.h"    // This is where a lot of the internals live
#include "math/point.hpp"

#include "a3/Constants.hpp"
//#include "a3/navigation.hpp"
//#include "a3/map.hpp"
#include "a3/LaserCorrector.hpp"
#include "a3/ParticleFilter.hpp"
#include "a3/Mapper.hpp"
#include "a3/SlamConstants.hpp"
#include "a3/RobotConstants.hpp"

#include "common/timestamp.h"


using namespace eecs467;

typedef struct state state_t;
struct state {
    	bool running;

    	getopt_t        *gopt;
    	parameter_gui_t *pg;

	image_u32_t image;


    	// vx stuff
    	vx_application_t    vxapp;
    	vx_world_t         *vxworld;      // where vx objects are live
    	vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    	vx_mouse_event_t    last_mouse_event;

	//map
//	Map map;

	//thread(s?)
	pthread_t lcm_thread;

    	// for accessing the arrays
    	pthread_mutex_t mutex;

	//lcm
	lcm::LCM* lcm;
	eecs467::LaserCorrector* laser;
        eecs467::ParticleFilter* pf;
        eecs467::Mapper* mapper;

	pacman_command_t cmd;

	maebot_pose_t pac_pose;

	bool key_down;

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
						lcm->publish("PACMAN_POSE", &pac_pose);
                       // nav->push_pose(newPose);

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



};//end state

void* lcm_thread(void* arg){
	
	int hz = 100;
	state_t* state = (state_t*) arg;
	while(state->running){
		state->lcm->handle();
		usleep(1000000 / hz);
	}
	return NULL;
}

static int
key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t* key)
{
	state_t *state = (state_t*)vxeh->impl;

	float key_pressed = 0.0;
	
	if(!key->released){
			state->key_down = true;
		if(key->key_code == VX_KEY_UP){
			std::cout << "PRESSED UP" << std::endl;
			key_pressed = UP;	
		}else if(key->key_code == VX_KEY_DOWN){
			std::cout << "PRESSED DOWN" << std::endl;
			key_pressed = DOWN;
		}else if(key->key_code == VX_KEY_RIGHT){
			std::cout << "PRESSED RIGHT" << std::endl;
			key_pressed = RIGHT;
		}else if(key->key_code == VX_KEY_LEFT){
			std::cout << "PRESSED LEFT" << std::endl;
			key_pressed = LEFT;
		}else if(key->key_code == 'q'){
			std::cout << "\n\nBYE BYE\n\n" << std::endl;
			key_pressed = END_PROG;
		}
	state->cmd.command = key_pressed;
	state->cmd.utime = utime_now();
	state->lcm->publish("PACMAN_COMMAND", &state->cmd);
	}else{
			//key_pressed = 1.0;	
			state->key_down = false;
	}
	
/*	state->cmd.command = key_pressed;
	state->cmd.utime = utime_now();
	state->lcm->publish("PACMAN_COMMAND", &state->cmd);
*/	
	return 0;
}

static int
mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{

    state_t *state = (state_t*)vxeh->impl;


    return 0;
}


state_t *
state_create (void)
{
    state_t *state = (state_t*)calloc (1, sizeof(*state));

    state->vxworld = vx_world_create ();
    state->vxeh = (vx_event_handler_t*)calloc (1, sizeof(*state->vxeh));
    state->vxeh->mouse_event = mouse_event;
    state->vxeh->key_event = key_event;
    state->vxeh->dispatch_order = 100;
    state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

    state->vxapp.display_started = eecs467_default_display_started;
    state->vxapp.display_finished = eecs467_default_display_finished;
    state->vxapp.impl = eecs467_default_implementation_create (state->vxworld, state->vxeh);

    state->running = 1;
	state->key_down = false;

    state->lcm = new lcm::LCM;

    
    if(!state->lcm->good()){
	std::cout << "ERROR INITIALIZING LCM!!!!!" << std::endl;
	exit(1);
    } 

        
        state->lcm->subscribe("PACMAN_LASER_SCAN",
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



    return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;


    if (state->vxeh)
        free (state->vxeh);

    if (state->gopt)
        getopt_destroy (state->gopt);

    if (state->pg)
        pg_destroy (state->pg);

    delete state->lcm;
        delete state->laser;
        delete state->pf;
        delete state->mapper;



	free (state);
}

int
main (int argc, char *argv[])
{
    eecs467_init (argc, argv);
    state_t *state = state_create ();

    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt,  'h', "help", 0, "Show help");
    getopt_add_bool   (state->gopt,  'l', "list", 0, "Lists available camera URLs and exit");
    getopt_add_string (state->gopt, 'f', "file", "", "Image Path");

    if (!getopt_parse (state->gopt, argc, argv, 1) || getopt_get_bool (state->gopt, "help")) {
        printf ("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage (state->gopt);
		std::cout 
			<<"\n\n\n// this program creates mask for overhead camera images.\n"
			<<"// if run with -f <path> loads picuture from file, else \n"
			<<"// it takes a snapshot and displays in gui. \n"            
			<<"//\n"
			<<"// click twice on image to form two bounding corners of rectangle.\n"                   
			<<"//    \n"
			<<"// masked image is shown to verify accuracy  \n"           
 			<<"// if mask is not acceptable click again to reset\n"
 			<<"//\n"
 			<<"// run from root directory so that image ends up in data\n"
 			<<"//\n"
 			<<"// to output mask:      \n"                                
 			<<"//   	click until acceptable mask   \n"                  
 			<<"//	press 's' to save                    \n"           
 			<<"//  	***again, run from root dir, so file goes to correct location\n"
 			<<"//\n"
 			<<"// output = X_lower, X_upper, Y_lower, Y_upper 		\n"
 			<<"////////////////////////////////////////////////////////\n"
 			<<"////////////////////////////////////////////////////////\n"
        	<< std::endl;
		exit (EXIT_FAILURE);
    }

	state->image = *(image_u32_create_from_pnm("data/pac-man.ppm"));
	
    // list option
    if (getopt_get_bool (state->gopt, "list")) {
        state_destroy (state);
        exit (EXIT_SUCCESS);
    }

    // launch lcm thread
    pthread_create (&state->lcm_thread, NULL, lcm_thread, state);


    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    // Initialize a parameter gui
    state->pg = pg_create ();


    // This is the main loop
    eecs467_gui_run (&state->vxapp, state->pg, 1024, 768);

    // Quit when GTK closes
    state->running = 0;

    // Cleanup
    pthread_join(state->lcm_thread, NULL);
    state_destroy (state);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();


}















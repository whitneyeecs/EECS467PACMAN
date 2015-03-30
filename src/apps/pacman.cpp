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

#include "eecs467_util.h"    // This is where a lot of the internals live

#include "a3/navigation.hpp"


typedef struct state state_t;
struct state {
    bool running;

    getopt_t        *gopt;
    parameter_gui_t *pg;

	image_u32_t image;

	Navigation* nav;

    // vx stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;      // where vx objects are live
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    vx_mouse_event_t    last_mouse_event;

    // threads
    pthread_t animate_thread;
	pthread_t command_thread;

    // for accessing the arrays
    pthread_mutex_t mutex;
};

static int
key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t* key)
{
	state_t *state = (state_t*)vxeh->impl;

	if(!key->released){
		if(key->key_code == VX_KEY_UP){
std::cout << "PRESSED UP" << std::endl;
			state->nav->go(UP);
std::cout <<"GOT OUT OF IT" << std::endl;
		}else if(key->key_code == VX_KEY_DOWN){
			state->nav->go(DOWN);
std::cout << "PRESSED DOWN" << std::endl;
		}else if(key->key_code == VX_KEY_RIGHT){
			state->nav->go(RIGHT);
std::cout << "PRESSED RIGHT" << std::endl;
		}else if(key->key_code == VX_KEY_LEFT){
			state->nav->go(LEFT);
std::cout << "PRESSED LEFT" << std::endl;
		}
	}else{
		state->nav->stop();
	}
	
	return 0;
}

static int
mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    state_t *state = (state_t*)vxeh->impl;


    return 0;
}

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
animate_thread (void *data)
{
    const int fps = 1;
    state_t *state = (state_t*)data;

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (state->running) {
		//ready to mask
break;
	//draw the image
	vx_object_t *vim = vxo_image_from_u32(&state->image,
               VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

	//render the image centered at the origin and 
	//at a normalized scale of +/-1 unit in x-dir
	const double scale = 2./state->image.width;
	vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "image"),
                            vxo_chain (vxo_mat_scale3 (scale, scale, 1.0),
                      		      vxo_mat_translate3 
				      (-state->image.width/2., -state->image.height/2., 0.),
                                       vim)
			     );
	vx_buffer_swap (vx_world_get_buffer (state->vxworld, "image"));
	fflush (stdout);


        usleep (1000000/fps);
    }


    return NULL;
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
	state->nav = new Navigation;
    
    // initialize bounds
    
    return state;
}

void
state_destroy (state_t *state)
{
    if (!state)
        return;

	delete state->nav;

    if (state->vxeh)
        free (state->vxeh);

    if (state->gopt)
        getopt_destroy (state->gopt);

    if (state->pg)
        pg_destroy (state->pg);

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

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    // Initialize a parameter gui
    state->pg = pg_create ();

    // Launch our worker threads
    pthread_create (&state->animate_thread, NULL, animate_thread, state);
    pthread_create (&state->command_thread, NULL, command_thread, state);

    // This is the main loop
    eecs467_gui_run (&state->vxapp, state->pg, 1024, 768);

    // Quit when GTK closes
    state->running = 0;
    pthread_join (state->animate_thread, NULL);

    // Cleanup
    state_destroy (state);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();


}















#include "user.h"
#include "common.h"
#include "ship.h"
#include <pthread.h>
#include "ptask.h"
#include <math.h>
#include <stdio.h>

struct timespec pressed;	//	time when ENTER is pressed + delay

//------------------------------------------------------------------------------
//	Reads the key pressed by the keyboard.
//	Only ESC, ENTER and SPACE BAR lead to reaction
//
// 	- ESC set the global variable end to true
//	- ENTER create a new ship and update pressed variable, once each delay sec.
//	- SPACE BAR set the gloabl variable show_route to true
//------------------------------------------------------------------------------
void button_pressed()
{
int delay = 500;//2000;		//	delay expressed in ms
char scan = 0;
struct timespec now;	//	the present time
int time_wakeup;		//	indicates if time pressed is expired

	clock_gettime(CLOCK_MONOTONIC, &now);
	time_wakeup = time_cmp(now, pressed);

	if (keypressed()) 
	{
		scan = readkey() >> 8;
	}
	
	switch (scan)
	{
		case KEY_ENTER:

				if (time_wakeup >= 0)
				{
					init_ship();				//	generates a new ship
					time_add_ms(&now, delay);	
					time_copy(&pressed, now);
				}
				break;

		case KEY_SPACE:
				
				pthread_mutex_lock(&mutex_s_route);
				show_routes = (show_routes) ? false : true;
				pthread_mutex_unlock(&mutex_s_route);
				break;

		case KEY_ESC:
					
				pthread_mutex_lock(&mutex_end);
				end = true;
				pthread_mutex_unlock(&mutex_end);
				break;
	}	
}

//------------------------------------------------------------------------------
//	Identifies the ship parked in the place clicked by the user.
//	If in that place there is no ship returns -1
//------------------------------------------------------------------------------
int find_parked()
{
int pos = -1;			//	parking place id
int ship_index = -1;	//	ship id
bool is_parked = false;

pos = click_place();	//	get the place id clicked by the user

	if (pos >= 0)
	{
		pthread_mutex_lock(&mutex_p);
		ship_index = places[pos].ship_id;	//	get the id of the parked ship
		pthread_mutex_unlock(&mutex_p);

		pthread_mutex_lock(&mutex_fleet);
		is_parked = fleet[ship_index].parking;
		pthread_mutex_unlock(&mutex_fleet);

		if (is_parked)	//	returns the ship id only if the ship was parked
		{
			return ship_index;
		}
	}
	return -1;
}

//	set the parking time of the ship clicked by the user to the current hourly
void woke_up()
{
struct timespec now;	//	the present time
int ship_index = find_parked();	//	get the id of the ship parked

	if (ship_index > -1)
	{
		clock_gettime(CLOCK_MONOTONIC, &now);
		pthread_mutex_lock(&mutex_fleet);
		fleet[ship_index].p_time = now;
		pthread_mutex_unlock(&mutex_fleet);

		printf("ship %d woke up\n", ship_index);
	}
}

//	add to the parking time of the ship clicked by the user "delay" ms more
void add_parking_time()
{
int ship_index = find_parked();	//	get the id of the parked ship clicked
int delay = 200;				//	delay that must be added to the parking time

	if (ship_index > -1)
	{
		pthread_mutex_lock(&mutex_fleet);
		time_add_ms(&fleet[ship_index].p_time, delay);
		printf("ship %d more %dms, actual time %ld\n", ship_index, delay,
							(fleet[ship_index].p_time).tv_sec);
		pthread_mutex_unlock(&mutex_fleet);
	}
}

//------------------------------------------------------------------------------
//	initialize the fields of the ship and its route having the indicated id.
//	i > -1: the ship already existed so will mantain its id.
//	i = -1: will be activated a new ship with new id and the global variable
//	ship_activated will be incremented.
//------------------------------------------------------------------------------
void initialize_ship(int i)
{
int id = (i > -1)? i : ships_activated;	//	ship id
int index = (id % ENTER_NUMBER);	//	indicates the enter trace assigned 

	pthread_mutex_lock(&mutex_fleet);
	fleet[id].parking = false;
	fleet[id].traj_grade = 3 * M_PI / 2;
	fleet[id].x = X_PORT * index; 
	fleet[id].y = PORT_BMP_H; 
	fleet[id].active = true;
	fleet[id].vel = 0;
	pthread_mutex_unlock(&mutex_fleet);

	pthread_mutex_lock(&mutex_route);	// each ship has linked a route
	routes[id].trace = enter_trace[index]; 
	routes[id].flip = (index == 2);	//	in this case the route must be flipped
	routes[id].index = -1;	//	the ship position index along the trace
	pthread_mutex_unlock(&mutex_route);

	if (id == ships_activated)
	{
		printf("ships_activated  %d  MAX_SHIPS %d\n", ships_activated + 1, 
																	MAX_SHIPS);
		
		//	if a new id has been used, updates the ship_activated variable	
		ships_activated += 1;	
		task_create(ship_task, PERIOD, DLINE, PRIO);	//	starts a new thread
	}

	else
	{
		printf("reassigned %d\n", i);
	}

}

//------------------------------------------------------------------------------
//	initializes new or reactivates a ship by calling initialize_ship
//	Will be generated first MAX_SHIPS ships with different id.
//	When MAX_SHIPS ships are been generated, will be reactivated an old ship
//	maintaining its id.
//------------------------------------------------------------------------------
void init_ship()
{
int i = 0;
bool active;	// state of a ship
bool reassigned = false;

	if (ships_activated < MAX_SHIPS)
	{
		initialize_ship(-1);	//	initialize a new ship
	}
	else
	{
		while (i < MAX_SHIPS && !reassigned)	
		{	
			pthread_mutex_lock(&mutex_fleet);
			active = fleet[i].active;		//	checks for a ship deactivated
			pthread_mutex_unlock(&mutex_fleet);
			
			if (!active)
			{
				reassigned = true;
				initialize_ship(i);	//	reactivate a ship
			}

			i += 1;
		}
	}
}

//	identifies the place id clicked by the user. Otherwise returns -1.
//	It exploits the global variable mouse_y & mouse_x to identify the place id.
int click_place()
{
int i;
int space;	//	left margin of a place
int half_num_parking = PLACE_NUMBER / 2;	//	number of places per side
int delta = 28;		//	place width
int offset = 19;	//	space between two places
int l_x = 121;		//	distance from left side to the first place on the x axis

//	distance from the right side to the fifth place on the x axis
int r_x = PORT_BMP_W - l_x - delta - (half_num_parking - 1) * (offset + delta);
	
	//	checks if mouse_y is in the range of the places
	if (mouse_y <= Y_PLACE && mouse_y >= Y_PLACE - YSHIP)
	{
		for (i = 0; i < half_num_parking; ++i)
		{
			space = i * (offset + delta);

			//	 checks if mouse_x is in the range of the i-th left place
			if(mouse_x <= l_x + space + delta && mouse_x >= l_x + space)
			{
				return i;	//	return the place id
			}

			//	 checks if mouse_x is in the range of the i-th right place
			else if (mouse_x <= r_x + space + delta && mouse_x >= r_x + space)
				 {
					return i + half_num_parking;	//	return the place id
				 }

		}
		return -1;	//	place not found
	}
	else return -1;	//	place not found
}

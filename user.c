#include "common.h"
#include "user.h"
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
int delay = 2000;	//	delay expressed in ms
char scan = 0;
struct timespec now;
int time_wakeup;	//	indicates if time pressed is expired

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
int pos = -1;
int ship_index = -1;
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
struct timespec now;
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
int id = (i > -1)? i : ships_activated;
int index = (id % ENTER_NUMBER);

	pthread_mutex_lock(&mutex_fleet);
	fleet[id].parking = false;
	fleet[id].traj_grade = 3 * M_PI / 2;
	fleet[id].x = X_PORT * index; 
	fleet[id].y = PORT_BMP_H; 
	fleet[id].active = true;
	fleet[id].vel = 0;
	pthread_mutex_unlock(&mutex_fleet);

	pthread_mutex_lock(&mutex_route);
	routes[id].trace = enter_trace[index]; 
	routes[id].flip = (index == 2);
	routes[id].index = -1;
	pthread_mutex_unlock(&mutex_route);

	if (id == ships_activated)
	{
		printf("ships_activated  %d  MAX_SHIPS %d\n", ships_activated + 1, MAX_SHIPS);
		ships_activated += 1;
		task_create(ship_task, PERIOD, DLINE, PRIO);
	}

	else
	{
		printf("reassigned %d\n", i);
	}

}

void init_ship()
{
int i;
bool active;

	if (ships_activated < MAX_SHIPS)
	{
		initialize_ship(-1);
	}
	else
	{
		for (i = 0; i < MAX_SHIPS; ++i)
		{	
			pthread_mutex_lock(&mutex_fleet);
			active = fleet[i].active;
			pthread_mutex_unlock(&mutex_fleet);
			if (!active)
			{
				initialize_ship(i);
				break;
			}
		}
	}
}

int click_place()
{
int i, space;
int half_num_parking = PLACE_NUMBER / 2;
int delta = 28;
int offset = 19;
int l_x = 121;
int r_x = PORT_BMP_W - l_x - delta - (half_num_parking - 1) * (offset + delta);

	if (mouse_y <= Y_PLACE && mouse_y >= Y_PLACE - YSHIP)
	{
		for (i = 0; i < half_num_parking; ++i)
		{
			space = i * (offset + delta);
			if(mouse_x <= l_x + space + delta && mouse_x >= l_x + space)
			{
				return i;
			}

			else if (mouse_x <= r_x + space + delta && mouse_x >= r_x + space)
				 {
					return i + half_num_parking;
				 }

		}
		return -1;
	}
	else return -1;
}

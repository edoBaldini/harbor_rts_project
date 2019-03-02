#include "common.h"
#include "user.h"
#include <pthread.h>
#include "ptask.h"
#include <math.h>
#include <stdio.h>

void * user_task(void * arg)
{   
int delay;
char scan;
bool c_end = false;
struct timespec pressed;
struct timespec delayed;
// Task private variables
const int id = get_task_index(arg);
	set_activation(id);
	clock_gettime(CLOCK_MONOTONIC, &pressed);

	while (!c_end) 
	{		
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		switch (mouse_b)
		{
			case 1:
					woke_up();
					break;

			case 2:
					add_parking_time();
					break;
		}

		delayed = botton_pressed(pressed);

	 	time_copy(&pressed, delayed);

		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! radar\n", id);
		}
		wait_for_activation(id);
	}

return NULL;
}

struct  timespec botton_pressed(struct timespec pressed)
{
int delay = 500;
char scan = 0;
struct timespec now;
int time_wakeup;

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
					init_ship();
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
	return pressed;
}

int find_parked()
{
int pos = -1;
int ship_index = -1;
int half_num_parking = N_PLACE / 2;
int delta = 28;
int offset = 19;
int l_x = 121;
int r_x = PORT_BMP_W - l_x - delta - (half_num_parking - 1) * (offset + delta);
bool is_parked = false;

pos = click_place(offset, delta, l_x, r_x);

	if (pos>= 0)
	{
		pthread_mutex_lock(&mutex_p);
		ship_index = places[pos].ship_id;
		pthread_mutex_unlock(&mutex_p);

		pthread_mutex_lock(&mutex_fleet);
		is_parked = fleet[ship_index].parking;
		pthread_mutex_unlock(&mutex_fleet);

		if (is_parked)
		{
			return ship_index;
		}
	}
	return -1;
}

void woke_up()
{
struct timespec now;
int ship_index = find_parked();

	if (ship_index > -1)
	{
		clock_gettime(CLOCK_MONOTONIC, &now);
		pthread_mutex_lock(&mutex_fleet);
		fleet[ship_index].p_time = now;
		pthread_mutex_unlock(&mutex_fleet);

		printf("ship %d woke up\n", ship_index);
	}
}

void add_parking_time()
{
int ship_index = find_parked();

	if (ship_index > -1)
	{
		pthread_mutex_lock(&mutex_fleet);
		time_add_ms(&fleet[ship_index].p_time, 200);
		printf("ship %d more 200ms, actual time %ld\n", ship_index, 
							(fleet[ship_index].p_time).tv_sec);
		pthread_mutex_unlock(&mutex_fleet);
	}
}

void initialize_ship(int i)
{
int id = (i > -1)? i : ships_activated;
int index = (id % 3);

	pthread_mutex_lock(&mutex_fleet);
	fleet[id].parking = false;
	fleet[id].traj_grade = 3 * M_PI / 2;
	fleet[id].x = 450 * index; 
	fleet[id].y = PORT_BMP_H; 
	fleet[id].active = true;
	fleet[id].vel = 0;
	pthread_mutex_unlock(&mutex_fleet);

	pthread_mutex_lock(&mutex_route);
	routes[id].trace = enter_trace[index]; 
	routes[id].odd = (index == 2);
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

int click_place(int offset, int delta, int l_x, int r_x)
{
int i, space;
int half_num_parking = N_PLACE / 2;

	if (mouse_y <= Y_PLACE && mouse_y >= Y_PLACE - YSHIP)
	{
		for (i = 0; i < half_num_parking; ++i)
		{
			space = i * (offset + delta);
			if(mouse_x <= l_x + space + delta && mouse_x >= l_x + space)
				return i;

			else if (mouse_x <= r_x + space + delta && mouse_x >= r_x + space)
					return i + half_num_parking;

		}
		return -1;
	}
	else return -1;
}

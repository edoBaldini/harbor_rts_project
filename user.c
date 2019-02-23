#include "common.h"
#include "user.h"
#include <pthread.h>
#include "ptask.h"
#include <math.h>
#include <stdio.h>

void * user_task(void * arg)
{   
char scan;
int time_passed;
struct timespec now;
struct timespec w_time;
bool c_end = false;

// Task private variables
const int id = get_task_index(arg);
	set_activation(id);

	while (!c_end) 
	{
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		clock_gettime(CLOCK_MONOTONIC, &now);
		time_passed = time_cmp(now, w_time);

		if (mouse_b == 1)
		{
			woke_up();
		}

		if (mouse_b == 2)
		{
			add_parking_time();
		}

		scan = 0;
		if (keypressed()) 
		{
			scan = readkey() >> 8;
		}

		if (scan == KEY_ENTER && time_passed >= 0)
		{
			clock_gettime(CLOCK_MONOTONIC, &w_time);
			time_add_ms(&w_time, 1500);

			init_ship();
		}

		if (scan == KEY_SPACE)
		{
			pthread_mutex_lock(&mutex_s_route);
			show_routes = (show_routes) ? false : true;
			pthread_mutex_unlock(&mutex_s_route);
		}

		if (scan == KEY_ESC)
		{
			pthread_mutex_lock(&mutex_end);
			end = true;
			pthread_mutex_unlock(&mutex_end);
			c_end = true;
		}

		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! radar\n", id);
		}
		wait_for_activation(id);
	}

return NULL;
}

int find_parked()
{
int pos = -1;
int ship_index = -1;
int half_num_parking = PLACE_NUMBER / 2;
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
int ship_index = find_parked();
	
	if (ship_index > -1)
	{
		pthread_mutex_lock(&mutex_fleet);
		fleet[ship_index].parking = false;
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

void init_ship()
{
int i;
int index = (ships_activated % 3);
bool active;

	if (ships_activated < MAX_SHIPS)
	{
		printf("ships_activated  %d  MAX_SHIPS %d\n", ships_activated, MAX_SHIPS);

		pthread_mutex_lock(&mutex_fleet);
		fleet[ships_activated].boat = create_bitmap(XSHIP, YSHIP);
		fleet[ships_activated].boat = load_bitmap("ship_c.bmp", NULL);
		fleet[ships_activated].parking = false;
		fleet[ships_activated].traj_grade = 3 * M_PI / 2;
		mark_label(fleet[ships_activated].boat);
		fleet[ships_activated].x = 0.0; 
		fleet[ships_activated].y = PORT_BMP_H - 1; 
		fleet[ships_activated].active = true;
		fleet[ships_activated].vel = MIN_VEL;
		pthread_mutex_unlock(&mutex_fleet);

		pthread_mutex_lock(&mutex_route);
		routes[ships_activated].x = X_PORT;
		routes[ships_activated].y = Y_PORT;
		routes[ships_activated].trace = enter_trace[index]; 
		routes[ships_activated].odd = (index == 1);
		pthread_mutex_unlock(&mutex_route);

		ships_activated += 1;

		task_create(ship_task, PERIOD, DLINE, PRIO);
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
				printf("reassigned %d\n", i);
				pthread_mutex_lock(&mutex_fleet);
				fleet[i].parking = false;
				fleet[i].traj_grade = 3 * M_PI / 2;
				fleet[i].x = 450 * (i % 3);
				fleet[i].y = PORT_BMP_H - 1;
				fleet[i].active = true;
				fleet[i].vel = MIN_VEL;
				pthread_mutex_unlock(&mutex_fleet);

				pthread_mutex_lock(&mutex_route);
				routes[i].x = X_PORT;
				routes[i].y = Y_PORT;
				routes[i].trace = enter_trace[i % 3]; 
				routes[i].odd = ((i % 3) == 1);
				pthread_mutex_unlock(&mutex_route);

				break;
			}
		}
	}
}
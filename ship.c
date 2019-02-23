#include "common.h"
#include "ship.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include "ptask.h"

static enum state {GUARD, PORT, PLACE, EXIT};

void * ship_task(void * arg)
{
int i;
int ship_id;
int guard_index, place_index, port_index, exit_index;
int cur_req;
int time_wakeup;
int color;
int objectives[6] = {YGUARD_POS, Y_PORT, Y_PLACE, Y_EXIT, -1, -2};
int index_objective = 0;
int step = GUARD;
bool mytrace_computed;
bool cur_repl;
bool wait;
bool termination;
bool is_odd;
bool move;
bool c_end;
BITMAP * cur_trace;
pair mytrace[X_PORT * Y_PORT];
ship cur_ship;

struct timespec now;

	// Task private variables
const int id = get_task_index(arg);
	set_activation(id);
	ship_id 			= id - AUX_THREAD;
	mytrace_computed 	= false;
	termination			= false;
	wait 				= false;
	c_end				= false;
	i 					= 0;
	while (!c_end) 
	{
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		pthread_mutex_lock(&mutex_fleet);
		cur_ship = fleet[ship_id];
		pthread_mutex_unlock(&mutex_fleet);

		clock_gettime(CLOCK_MONOTONIC, &now);
		time_wakeup = time_cmp(now, cur_ship.p_time);

		pthread_mutex_lock(&mutex_route);
		cur_trace = routes[ship_id].trace;
		is_odd = routes[ship_id].odd;
		pthread_mutex_unlock(&mutex_route);

		pthread_mutex_lock(&mutex_rr);
		cur_repl = reply_access[ship_id];
		cur_req = request_access[ship_id];
		pthread_mutex_unlock(&mutex_rr);
		if (cur_ship.active)
		{ 
			move = (check_forward(cur_ship.x, cur_ship.y, cur_ship.traj_grade)) ? false: true;
			if (step == GUARD)
			{	
				if (!mytrace_computed)
				{
					i = 0;
					guard_index = compute_mytrace(ship_id, is_odd, mytrace, cur_trace, YGUARD_POS);
					mytrace_computed = true;
				}

				if(check_position(cur_ship.y, YGUARD_POS))
				{
					cur_repl = false;
					cur_req = Y_PORT;
					i = guard_index;
					step = PORT;
					mytrace_computed = false;
				}

				else if (move)
				{
					i = follow_track_frw(ship_id, XGUARD_POS, YGUARD_POS, move, cur_trace);
				}
				else
				{
					i = follow_track_frw(ship_id, XGUARD_POS, YGUARD_POS, move, cur_trace);	
				}
			}

			if (step == PORT && cur_repl)
			{
				if (!mytrace_computed)
				{
					port_index = find_index(mytrace, Y_PORT);
					mytrace_computed = true;
				}
				if(check_position(cur_ship.y, Y_PORT))
				{
					cur_repl = false;
					cur_req = Y_PLACE;
					i = 0;
					mytrace_computed = false;
					step = PLACE;
				}

				else 
				{
					i = follow_track_frw(ship_id, X_PORT, Y_PORT, true, cur_trace);
				}			
			}

			if (step == PLACE && cur_repl)
			{
				if (!mytrace_computed)
				{
					place_index = compute_mytrace(ship_id, is_odd, mytrace, cur_trace, Y_PLACE - YSHIP);
					mytrace_computed = true;
				}

				if (check_position(cur_ship.y, Y_PLACE + XSHIP))
				{
					cur_req = 1;
				}

				if(check_position(cur_ship.y, Y_PLACE - YSHIP))
				{
					if (!wait)
					{	
						cur_req = 1;
						cur_ship.parking = true;
						pthread_mutex_lock(&mutex_fleet);
						clock_gettime(CLOCK_MONOTONIC, &fleet[ship_id].p_time);
						time_add_ms(&fleet[ship_id].p_time, random_in_range(MIN_P_TIME, MAX_P_TIME));
						pthread_mutex_unlock(&mutex_fleet);

						wait = true;
					}
					else
					{
						if (time_wakeup >= 0 || !cur_ship.parking)
						{
							cur_ship.parking = false;
							cur_repl = false;
							cur_req = Y_EXIT;
							wait = false;
							i = 0;
							mytrace_computed = false;
							step = EXIT;
						}
					}
				}

				else
				{

					i = follow_track_frw(ship_id, X_PORT, Y_PLACE, true, cur_trace);
				}

			}

			if (step == EXIT && cur_repl)
			{
				if (!mytrace_computed)
				{
					exit_index = compute_mytrace(ship_id, is_odd, mytrace, cur_trace, Y_EXIT);
					mytrace_computed = true;;
				}
				if (cur_ship.y < mytrace[0].y)
					rotate90_ship(ship_id, cur_ship.x,Y_PLACE, mytrace[0].y + YSHIP);

				else if (cur_ship.x > EPSILON + YSHIP && cur_ship.x < PORT_BMP_W - EPSILON - YSHIP)
				{
					i = follow_track_frw(ship_id, X_PORT, Y_EXIT, true, cur_trace);
				}

				if (check_position(cur_ship.y, Y_PORT - XSHIP) && cur_req == Y_EXIT)
				{
					cur_req = -1;
				}

				if (cur_ship.x <= EPSILON + YSHIP|| cur_ship.x >= PORT_BMP_W - EPSILON - YSHIP)
				{
					termination = exit_ship(ship_id, cur_ship.x);
					if (termination)
					{
						mytrace_computed = false;
						cur_req = YGUARD_POS;
						cur_repl = false;
						step = GUARD;
						cur_ship.active = false;
						cur_trace = NULL;
					}
				}
			}	

			pthread_mutex_lock(&mutex_fleet);
			fleet[ship_id].active = cur_ship.active;
			fleet[ship_id].parking = cur_ship.parking;
			pthread_mutex_unlock(&mutex_fleet);

			pthread_mutex_lock(&mutex_rr);
			reply_access[ship_id] = cur_repl;
			request_access[ship_id] = cur_req;
			pthread_mutex_unlock(&mutex_rr);
			
			pthread_mutex_lock(&mutex_route);
			routes[ship_id].trace = cur_trace;
			pthread_mutex_unlock(&mutex_route);
		}

		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! ship\n", id);
		}
		wait_for_activation(id);
	}


	return NULL;
 }

//------------------------------------------------------------------------------
// FUNCTIONS FOR SHIPS
//------------------------------------------------------------------------------
int compute_mytrace(int ship_id, bool is_odd, pair mytrace[X_PORT * Y_PORT], 
																	BITMAP * cur_trace, int obj)
{
int index_objective;

	make_array_trace(cur_trace, mytrace, ship_id, is_odd, obj);
	index_objective =  find_index(mytrace, obj);
	return index_objective;
}

void reverse_array(pair trace[X_PORT * Y_PORT], int last_index)
{
int i;
pair aux;
int size = last_index;
	for(i = 0; i < size / 2; ++i)
	{
		aux = trace[i];
		trace[i] = trace[last_index];
		trace[last_index] = aux;
		last_index --;
	}
}

void make_array_trace(BITMAP * t, pair trace[PORT_BMP_W * PORT_BMP_H], int id, bool odd, int obj)
{
int color;
int index = 0;
int i, j;
int last_index;
for (j = PORT_BMP_H; j > 0; --j)   
	{
		if (odd)
		{
			for (i = PORT_BMP_W; i > 0 ; --i)
			{
				color = getpixel(t, i, j);
				if (color == 0 || color == makecol(255, 0, 0))
				{
					trace[index] = make_pair(i,j);
					index ++;
				}
			}
		}
		else
		{	
			for (i = 0; i < PORT_BMP_W; ++i)
			{
				int color = getpixel(t, i, j);
				if (color == 0 || color == (makecol(255,0,0)))
				{
					trace[index] = make_pair(i,j);
					index ++;
				}
			}
		}
	}
	last_index = -- index;

	if (obj == Y_EXIT)
		reverse_array(trace, last_index);
}

bool check_forward(float x_cur, float y_cur, float g_cur)
{
float x, y, j;
int color1, color2;

	for (j = (YSHIP/2) + 2; j < 90; ++j )
	{
		x = x_cur + j * cos(g_cur);
		y = y_cur + j * sin(g_cur) + (YSHIP / 2 );
		pthread_mutex_lock(&mutex_sea);
		color1 = getpixel(sea, x, y);
		color2 = getpixel(sea, x + (XSHIP / 2) ,y - (XSHIP / 2));
		pthread_mutex_unlock(&mutex_sea);

		if ((color1 != SEA_COLOR && color1 != -1) ||
				color2 != SEA_COLOR && color2 != -1 )
		{
			return true; 		
		}
	}	
	
 	return false;
}

bool check_position(float y_ship, int y)
{
	return fabs(y_ship - y) <= EPSILON;
}

float update_vel(float vel, int color, float des)
{
	vel = (color == 0)? vel + 0.001 : vel - 0.001;

	if (vel > MAX_VEL)
	{
		vel = MAX_VEL;
	}

	if (vel < MIN_VEL)
	{
		vel = MIN_VEL;
	}

	if (vel > des)
	{
		vel = des;
	}

	return vel;
}

void grade_filter(int id, int i, pair mytrace[X_PORT * Y_PORT])
{
float p = powf(M_E,(-0.115129 * PERIOD));
float grade = p * (degree_rect(fleet[id].x, fleet[id].y, mytrace[i].x,
						 mytrace[i].y)) + (1 - p) * (fleet[id].traj_grade);
fleet[id].traj_grade = grade;

}

void grade_filter_b(int id, float x, float y)
{
//float p = powf(M_E,(-0.115129 * PERIOD));
float p = 0.03;
float grade = p * (degree_rect(fleet[id].x, fleet[id].y, x,
						y)) + (1 - p) * (fleet[id].traj_grade);
fleet[id].traj_grade = grade;

}

int follow_track_frw(int id, int objx, int objy, bool move, BITMAP * cur_trace)
{
float vel, velx, vely;
float x, y;
float des;
float alpha = (-1) * (M_PI / 2); 
float p = 0.6;
int red = makecol(255, 0, 0);
int color;
	
	des = fabs(objy - fleet[id].y + (YSHIP / 2 )) / 2;
	
	if(id ==1)
	{
		printf("vel %f\n", fleet[id].vel);
		printf("%f\n", des);
	}

	while (alpha < (M_PI / 2))
	{
		velx = fleet[id].vel * cos(fleet[id].traj_grade + alpha);
		vely = fleet[id].vel * sin(fleet[id].traj_grade + alpha);

		x = fleet[id].x + velx * PERIOD;
		y = fleet[id].y + vely * PERIOD;
		vel = fleet[id].vel;

		color = getpixel(cur_trace, x, y);
		if (color == 0 || color == red)
		{
			fleet[id].vel = update_vel(vel, color, des);
			if (move)
			{
				grade_filter_b(id, x, y);
				fleet[id].x = p * x + (1 - p) * fleet[id].x;
				fleet[id].y = p * y + (1 - p) * fleet[id].y;
				break;
			}
			else
			{
				fleet[id].x = p * fleet[id].x + (1 - p) * fleet[id].x;
				fleet[id].y = p * fleet[id].y + (1 - p) * fleet[id].y;
				break;				
			}
		}

		alpha += 0.0001;
	}

	pthread_mutex_unlock(&mutex_fleet);
	
	return index;
}
void rotate90_ship(int id, float x_cur, int y1, int y2)
{
float aux = 10000 / PERIOD;
	
	if (x_cur > X_PORT)
	{
		pthread_mutex_lock(&mutex_fleet);
		fleet[id].y += (y2 - y1) / aux;
		fleet[id].traj_grade -= (M_PI / 2) / aux;
		pthread_mutex_unlock(&mutex_fleet);
	}
	else
	{
		pthread_mutex_lock(&mutex_fleet);
		fleet[id].y += (y2 - y1) / aux;
		fleet[id].traj_grade += (M_PI / 2) / aux;
		pthread_mutex_unlock(&mutex_fleet);
	}
}

bool exit_ship(int id, float x_cur)
{
float aux = 1000 / PERIOD;
	if(x_cur < X_PORT)
	{
		if (x_cur > -YSHIP)
		{
			pthread_mutex_lock(&mutex_fleet);
			fleet[id].x -= YSHIP / aux;
			fleet[id].traj_grade = M_PI;
			pthread_mutex_unlock(&mutex_fleet);

			return false;
		}
		else return true;
	}
	else
	{	if (x_cur < PORT_BMP_W + YSHIP)
		{	
			pthread_mutex_lock(&mutex_fleet);
			fleet[id].x += YSHIP/ aux;
			fleet[id].traj_grade = 2 * M_PI;
			pthread_mutex_unlock(&mutex_fleet);
			return false;
		}
		else return true;
	}
}

float distance_vector (float x1, float y1, float x2, float y2)
{
	return sqrtf(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
}

int find_index(pair mytrace[X_PORT * Y_PORT], int posix)
{
int i;
	for (i = 0; i < PORT_BMP_W * PORT_BMP_H; ++i)
	{
		if (mytrace[i].y == posix)
			return i;
	}
}
#include "common.h"
#include "ship.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include "ptask.h"

void * ship_task(void * arg)
{
int i;
int ship_id;
int guard_index, place_index, port_index, exit_index;
int cur_req;
int time_wakeup;
int color;
bool mytrace_computed;
bool first_step, second_step, third_step, fourth_step;
bool cur_repl;
bool wait;
bool termination;
bool active;
bool is_parked;
bool is_odd;
bool move;
bool c_end;
float x_cur, y_cur, g_cur;

BITMAP * cur_trace;
pair mytrace[X_PORT * Y_PORT];

struct timespec now;

	// Task private variables
const int id = get_task_index(arg);
	set_activation(id);
	ship_id 			= id - AUX_THREAD;
	first_step			= true;
	second_step 		= false;
	third_step 			= false;
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
		x_cur = fleet[ship_id].x;
		y_cur = fleet[ship_id].y;
		g_cur = fleet[ship_id].traj_grade;
		active = fleet[ship_id].active;
		is_parked = fleet[ship_id].parking;
		clock_gettime(CLOCK_MONOTONIC, &now);
		time_wakeup = time_cmp(now, fleet[ship_id].p_time);
		pthread_mutex_unlock(&mutex_fleet);

		pthread_mutex_lock(&mutex_route);
		cur_trace = routes[ship_id].trace;
		is_odd = routes[ship_id].odd;
		pthread_mutex_unlock(&mutex_route);

		pthread_mutex_lock(&mutex_rr);
		cur_repl = reply_access[ship_id];
		cur_req = request_access[ship_id];
		pthread_mutex_unlock(&mutex_rr);

		if (active)
		{
			move = (check_forward(x_cur, y_cur, g_cur)) ? false: true;

			if (first_step)
			{	
				if (!mytrace_computed)
				{
					guard_index = compute_mytrace(ship_id, is_odd, mytrace, cur_trace, YGUARD_POS);
					mytrace_computed = true;
				}

				if(check_position(y_cur, YGUARD_POS))
				{
					cur_repl = false;
					cur_req = Y_PORT;
					i = guard_index;
					second_step = true;
					first_step = false;
					mytrace_computed = false;
				}

				else if (move)
				{
					i = follow_track_frw(ship_id, i, mytrace, guard_index, move, cur_trace);
				}
				else
				{
					i = follow_track_frw(ship_id, i, mytrace, i, move, cur_trace);	
				}
			}

			if (second_step && cur_repl)
			{
				if (!mytrace_computed)
				{
					port_index = find_index(mytrace, Y_PORT);
					mytrace_computed = true;
				}
				if(check_position(y_cur, Y_PORT))
				{
					cur_repl = false;
					cur_req = Y_PLACE;
					i = 0;
					mytrace_computed = false;
					third_step = true;
					second_step = false;
					first_step = false;
				}

				else 
				{
					i = follow_track_frw(ship_id, i, mytrace, port_index, true, cur_trace);
				}			
			}

			if (third_step && cur_repl)
			{
				if (!mytrace_computed)
				{
					place_index = compute_mytrace(ship_id, is_odd, mytrace, cur_trace, Y_PLACE - YSHIP);
					mytrace_computed = true;
				}

				if (check_position(y_cur, Y_PLACE + XSHIP))
				{
					cur_req = 1;
				}

				if(check_position(y_cur, Y_PLACE - YSHIP))
				{
					if (!wait)
					{	
						cur_req = 1;
						is_parked = true;
						pthread_mutex_lock(&mutex_fleet);
						clock_gettime(CLOCK_MONOTONIC, &fleet[ship_id].p_time);
						time_add_ms(&fleet[ship_id].p_time, random_in_range(MIN_P_TIME, MAX_P_TIME));
						pthread_mutex_unlock(&mutex_fleet);

						wait = true;
					}
					else
					{
						if (time_wakeup >= 0 || !is_parked)
						{
							is_parked = false;
							cur_repl = false;
							cur_req = Y_EXIT;
							wait = false;
							i = 0;
							mytrace_computed = false;
							third_step = false;
							fourth_step = true;
						}
					}
				}

				else
				{

					i = follow_track_frw(ship_id, i, mytrace, place_index, true, cur_trace);
				}

			}

			if (fourth_step && cur_repl)
			{
				if (!mytrace_computed)
				{
					exit_index = compute_mytrace(ship_id, is_odd, mytrace, cur_trace, Y_EXIT);
					mytrace_computed = true;;
				}
				if (y_cur < mytrace[0].y)
					rotate90_ship(ship_id, x_cur,Y_PLACE, mytrace[0].y + YSHIP);

				else if (x_cur > EPSILON + YSHIP && x_cur < PORT_BMP_W - EPSILON - YSHIP)
				{
					i = follow_track_frw(ship_id, i, mytrace, exit_index, true, cur_trace);
				}

				if (check_position(y_cur, Y_PORT - XSHIP) && cur_req == Y_EXIT)
				{
					cur_req = -1;
				}

				if (x_cur <= EPSILON + YSHIP|| x_cur >= PORT_BMP_W - EPSILON - YSHIP)
				{
					termination = exit_ship(ship_id, x_cur);
					if (termination)
					{
						mytrace_computed = false;
						cur_req = YGUARD_POS;
						cur_repl = false;
						first_step = true;
						fourth_step = false;
						active = false;
						cur_trace = NULL;
					}
				}
			}	

			pthread_mutex_lock(&mutex_fleet);
			fleet[ship_id].active = active;
			fleet[ship_id].parking = is_parked;
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


void grade_filter(int id, int i, pair mytrace[X_PORT * Y_PORT])
{
float p = powf(M_E,(-0.115129 * PERIOD));
float grade = p * (degree_rect(fleet[id].x, fleet[id].y, mytrace[i].x,
						 mytrace[i].y)) + (1 - p) * (fleet[id].traj_grade);
fleet[id].traj_grade = grade;

}


int follow_track_frw(int id, int i, pair mytrace[X_PORT * Y_PORT], 
										int last_index, bool move, BITMAP * cur_trace)
{
	float p = 0.03;
	float des;
	float acc;
	float d_obj;
	float c_vel;
	int red = makecol(255, 0, 0);
	int index;

	pthread_mutex_lock(&mutex_fleet);
	c_vel = fleet[id].vel;
	d_obj = distance_vector(fleet[id].x, fleet[id].y, mytrace[last_index].x, 
												mytrace[last_index].y);
	
	c_vel = ((d_obj / 2) * p < c_vel) ? (d_obj / 2) * p : c_vel;
	c_vel = (c_vel > MAX_VEL) ? MAX_VEL: c_vel;

	fleet[id].vel = (fleet[id].vel < c_vel) ? fleet[id].vel: c_vel;

	if (getpixel(cur_trace, mytrace[i].x, mytrace[i].y) == red)
	{
		fleet[id].vel -= 0.03;
		fleet[id].vel = (fleet[id].vel < MIN_VEL)? fleet[id].vel + MIN_VEL: fleet[id].vel;
	}

	else 
	{
		fleet[id].vel += 0.03;
	}

	des = fleet[id].vel*PERIOD;
	acc = distance_vector(fleet[id].x, fleet[id].y, mytrace[i].x, mytrace[i].y);

	while (des > acc)
	{
		i += 1;
		acc += distance_vector(fleet[id].x, fleet[id].y, mytrace[i].x, 
																mytrace[i].y);
	}

	index = (i > last_index) ? last_index: i;
	fleet[id].x = p * (mytrace[index].x ) + (1 - p) * (fleet[id].x);
	fleet[id].y = p * (mytrace[index].y ) + (1 - p) * (fleet[id].y);

	if (move)
		grade_filter(id, index, mytrace);

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
#include "harbor.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include "ptask.h"

#define MIN_VEL			3		// minimum speed
#define	MAX_VEL			3		// maximum speed

void * ship_task(void * arg)
{
int i;
int ship_id;
int last_index, guard_index, place_index;
int cur_req;
int time_wakeup;
int color;
int t, w;
int red;
bool mytrace_computed;
bool first_step, second_step, third_step, fourth_step;
bool cur_repl;
bool wait;
bool termination;
bool active;
bool is_parked;
bool is_odd;
bool move;
float x_cur, y_cur, g_cur;
float vel;

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
	i 					= 0;
	red					= makecol(255, 0,0);

	while (!end) 
	{
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
					last_index = make_array_trace(cur_trace, mytrace, 
													ship_id, is_odd, cur_req);
					mytrace_computed = true;
					i = 0;
					guard_index = find_index(mytrace, YGUARD_POS);
					pthread_mutex_lock(&mutex_fleet);
					fleet[ship_id].x = mytrace[i].x;
					fleet[ship_id].y = mytrace[i].y;
					pthread_mutex_unlock(&mutex_fleet);
				}

				if(check_position(y_cur, YGUARD_POS))
				{
					cur_repl = false;
					cur_req = Y_PORT;
					i = guard_index;
					w = 0;
					t = 0;
					second_step = true;
					first_step = false;
				}

				else if (move)
				{
					color = getpixel(cur_trace, mytrace[i].x, mytrace[i].y);
					
					if (color == red)
					{
						t = 0;
						w += 1;
						vel += (1 - powf(M_E,(0.0005 * w)));
						vel = (vel < MIN_VEL) ? MIN_VEL : vel;
					}
					else
					{
						t +=1;
						w = 0;
						vel -= (1 - powf(M_E,(0.0005 * t)));
						vel = (vel > MAX_VEL) ? MAX_VEL : vel;

					}
					i = follow_track_frw(ship_id, i, mytrace, guard_index, move, vel);
				}
				else
				{
						t = 0;
						w += 1;
						vel += (1 - powf(M_E,(0.0005 * w)));					
				}

			}

			if (second_step && cur_repl)
			{
				if(check_position(y_cur, Y_PORT))
				{
					cur_repl = false;
					cur_req = Y_PLACE;
					i = 0;
					mytrace_computed = false;
					third_step = true;
					second_step = false;
					first_step = false;
					vel = MIN_VEL;
				}

				else 
				{
					color = getpixel(cur_trace, mytrace[i].x, mytrace[i].y);

					if (color == red)
					{
						t = 0;
						w += 1;
						vel += (1 - powf(M_E,(0.0005 * w)));
						vel = (vel < MIN_VEL) ? MIN_VEL : vel;
					}
					else
					{
						t +=1;
						w = 0;
						vel -= (1 - powf(M_E,(0.0005 * t)));
						vel = (vel > MAX_VEL) ? MAX_VEL : vel;

					}
					i = follow_track_frw(ship_id, i, mytrace, last_index, true, vel);
				}

			
			}

			if (third_step && cur_repl)
			{
				if (!mytrace_computed)
				{
					last_index = make_array_trace(cur_trace, mytrace, 
													ship_id, is_odd, cur_req);
					mytrace_computed = true;
					i = 0;
					place_index = find_index(mytrace, Y_PLACE - YSHIP);
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
							mytrace_computed = false;
							t = 0;
							w = 0;
							third_step = false;
							fourth_step = true;
						}
					}
				}

				else
				{

					color = getpixel(cur_trace, mytrace[i].x, mytrace[i].y);
					
					if (color == red)
					{
						t = 0;
						w += 1;
						vel += (1 - powf(M_E,(0.0005 * w)));
						vel = (vel < MIN_VEL) ? MIN_VEL : vel;
					}
					else
					{
						t +=1;
						w = 0;
						vel -= (1 - powf(M_E,(0.0005 * t)));
						vel = (vel > MAX_VEL) ? MAX_VEL : vel;

					}

					i = follow_track_frw(ship_id, i, mytrace, place_index, true, vel);
				}

			}

			if (fourth_step && cur_repl)
			{
				if (!mytrace_computed)
				{
					last_index = make_array_trace(cur_trace, mytrace,
					 								ship_id, is_odd, cur_req);
					mytrace_computed = true;
					printf("y of exit %f\n", mytrace[last_index].y);
					i = 0;
				}
				if (y_cur < mytrace[0].y)
					rotate90_ship(ship_id, x_cur,Y_PLACE, mytrace[0].y + YSHIP);

				else if (x_cur > EPSILON + YSHIP && x_cur < PORT_BMP_W - EPSILON - YSHIP)
				{
					color = getpixel(cur_trace, mytrace[i].x, mytrace[i].y);
					
					if (color == red)
					{
						t = 0;
						w += 1;
						vel += (1 - powf(M_E,(0.0005 * w)));
						vel = (vel < MIN_VEL) ? MIN_VEL : vel;
					}
					else
					{
						t +=1;
						w = 0;
						vel -= (1 - powf(M_E,(0.0005 * t)));
						vel = (vel > MAX_VEL) ? MAX_VEL : vel;

					}

					i = follow_track_frw(ship_id, i, mytrace, last_index, true, vel);
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
						t = 0;
						w = 0;
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

int make_array_trace(BITMAP * t, pair trace[PORT_BMP_W * PORT_BMP_H], int id, 
															bool odd, int req)
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

	if (req == Y_EXIT)
		reverse_array(trace, last_index);

	return last_index;
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
										int last_index, bool move, float vel)
{
	float p = 0.03;
	float des;
	float acc;
	float d_obj;
	int index;

	pthread_mutex_lock(&mutex_fleet);

	d_obj = distance_vector(fleet[id].x, fleet[id].y, mytrace[last_index].x, 
												mytrace[last_index].y);
	vel = ((d_obj / 2) * p < vel) ? (d_obj / 2) * p : vel;
	des = vel*PERIOD;
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
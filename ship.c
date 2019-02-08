#include "harbor.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include "ptask.h"

void * ship_task(void * arg)
{

int i;
int ship_id;
int last_index;
int cur_req;
int time_wakeup;
pair mytrace[X_PORT * Y_PORT];
struct timespec now;
bool mytrace_computed;
bool first_step, second_step, third_step, fourth_step;
bool cur_repl;
bool move = true;
bool wait = false;
bool termination = false;
bool active;
bool is_parked;
bool is_odd;
float x_cur, y_cur, g_cur;
BITMAP * cur_trace;

	// Task private variables
const int id = get_task_index(arg);
	set_activation(id);
	ship_id 			= id - AUX_THREAD;
	first_step			= true;
	second_step 		= false;
	third_step 			= false;
	fourth_step			= false;
	mytrace_computed 	= false;
	i 					= 0;

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
				}

				if(check_yposition(y_cur, YGUARD_POS))
				{
					cur_req = Y_PORT;
					second_step = true;
					first_step = false;
				}

				else if (move)
				{
					follow_track_frw(ship_id, i, mytrace, last_index);
					i++;
				}

			}

			if (second_step && cur_repl)
			{
				if(check_yposition(y_cur, Y_PORT))
				{
					cur_repl = false;
					cur_req = Y_PLACE;
					i = 0;
					mytrace_computed = false;
					third_step = true;
					second_step = false;
				}

				else
				{
					follow_track_frw(ship_id, i, mytrace, last_index);
					i++;
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
				}

				if(check_yposition(y_cur, Y_PLACE - YSHIP))
				{

					if (!wait)
					{	
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
							mytrace_computed = false;
							third_step = false;
							fourth_step = true;
						}
					}
				}

				else if(move)
				{
					follow_track_frw(ship_id, i, mytrace, last_index);
					i++;
				}

			}

			if (fourth_step && cur_repl)
			{
				if (!mytrace_computed)
				{
					last_index = make_array_trace(cur_trace, mytrace,
					 								ship_id, is_odd, cur_req);
					mytrace_computed = true;
					i = 0;
				}
				if (y_cur < mytrace[0].y)
					rotate90_ship(ship_id, x_cur,Y_PLACE, mytrace[0].y + YSHIP);

				else
				{
					follow_track_frw(ship_id, i, mytrace, last_index);
					i++;
				}

				if (check_yposition(y_cur, Y_PORT - XSHIP) && 
													cur_req != -1)
				{
					cur_repl = false;
					cur_req = -1;
				}

				if (cur_req == -1 && cur_repl)
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
				if (color == 0)
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
				if (color == 0)
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

	//routes[id].x = trace[last_index].x;
	return last_index;
}

bool check_forward(float x_cur, float y_cur, float g_cur)
{
int x, y, j;
int color;

	for (j = 2; j < 70; ++j )
	{
		x = x_cur + j * cos(g_cur);
		y = y_cur + j * sin(g_cur);
		color = getpixel(sea, x, y);
		if (color != SEA_COLOR && color != -1)
			return true; 		
	}
	
 	return false;
}

bool check_yposition(float y_ship, int y)
{
	return fabs(y_ship - y) <= EPSILON;
}

void follow_track_frw(int id, int i, pair mytrace[X_PORT * Y_PORT], 
																int last_index)
{
	if(i <= last_index)
	{
		pthread_mutex_lock(&mutex_fleet);
		fleet[id].x = mytrace[i].x;
		fleet[id].y = mytrace[i].y;

		if (last_index > i + 60)
			fleet[id].traj_grade = degree_rect(fleet[id].x, fleet[id].y, 
						mytrace[i + 20].x, mytrace[i + 20].y);
		else 
			fleet[id].traj_grade = fleet[id].traj_grade;
		pthread_mutex_unlock(&mutex_fleet);
	}
}

void rotate90_ship(int id, float x_cur, int y1, int y2)
{
float aux = 5000 / PERIOD;
	
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
			pthread_mutex_unlock(&mutex_fleet);
			return false;
		}
		else return true;
	}
}

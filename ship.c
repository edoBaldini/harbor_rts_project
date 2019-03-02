#include "common.h"
#include "ship.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include "ptask.h"

enum state {GUARD, PORT, PLACE, EGRESS};

enum state reach_guard(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool move)
{
int index;
enum state step;
bool guard_reached = check_position(cur_ship.y, YGUARD_POS);
	pthread_mutex_lock(&mutex_route);
	index = routes[ship_id].index;
	pthread_mutex_unlock(&mutex_route);

	if(guard_reached)
	{
		index = find_index(mytrace, Y_PORT);
		step = update_state(ship_id, PORT, Y_PORT, index);
		return step;
	}
	follow_track_frw(ship_id, move);
	return GUARD;
}

enum state reach_port(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool move)
{
enum state step;	
bool cur_repl = get_repl(ship_id);	
bool port_reached = check_position(cur_ship.y, Y_PORT);

	if (cur_repl)
	{
		if(port_reached)
		{
			step = update_state(ship_id, PLACE, Y_PLACE, 0);
			return step;
		}
		else 
		{
			follow_track_frw(ship_id, true);
		}
	}

	return PORT;
}

enum state reach_place(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool move)
{
int index;
enum state step;
int time_wakeup;
struct timespec now;
bool cur_repl = get_repl(ship_id);
bool place_reached = check_position(cur_ship.y, Y_PLACE + XSHIP);
bool parked = check_position(cur_ship.y, Y_PLACE - YSHIP);
	
	pthread_mutex_lock(&mutex_route);
	index = routes[ship_id].index;
	pthread_mutex_unlock(&mutex_route);
	
	clock_gettime(CLOCK_MONOTONIC, &now);
	time_wakeup = time_cmp(now, cur_ship.p_time);
	
	if (cur_repl)
	{

		if (place_reached)
		{
			update_rr(ship_id, cur_repl, 1);
		}

		if(parked)
		{
			if (!cur_ship.parking)
			{	
				cur_ship.parking = true;
				pthread_mutex_lock(&mutex_fleet);
				clock_gettime(CLOCK_MONOTONIC, &fleet[ship_id].p_time);
				time_add_ms(&fleet[ship_id].p_time, random_in_range(MIN_P_TIME, MAX_P_TIME));
				fleet[ship_id].parking = true;
				pthread_mutex_unlock(&mutex_fleet);
			}
			else
			{
				if (time_wakeup >= 0)
				{
					pthread_mutex_lock(&mutex_fleet);
					fleet[ship_id].parking = false;
					pthread_mutex_unlock(&mutex_fleet);

					cur_ship.parking = false;
					step = update_state(ship_id, EGRESS, Y_EXIT, 0);
					return step;
				}
			}
		}
		else
		{	
			follow_track_frw(ship_id, true);
		}
	}
	return PLACE;	
}

enum state reach_exit(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool move)
{
int index;
enum state step;
bool cur_repl = get_repl(ship_id);
bool cur_req = get_req(ship_id);
bool exit_reached = check_position(cur_ship.y, Y_PORT - XSHIP) && cur_req == Y_EXIT;
	
	pthread_mutex_lock(&mutex_route);
	index = routes[ship_id].index;
	pthread_mutex_unlock(&mutex_route);
	
	if (cur_repl)
	{ 

		if (cur_ship.y < mytrace[0].y)
		{
			rotate90_ship(ship_id, cur_ship.x, Y_PLACE, mytrace[0].y + YSHIP);
		}
		else if (cur_ship.x <= EPSILON + YSHIP || cur_ship.x >= PORT_BMP_W - EPSILON - YSHIP)
		{
			exit_ship(ship_id, cur_ship.x);
			if (cur_ship.x < -YSHIP || cur_ship.x > PORT_BMP_W + YSHIP)
			{
				step = update_state(ship_id, GUARD, YGUARD_POS, 0);
				cur_ship.active = false;

				pthread_mutex_lock(&mutex_fleet);
				fleet[ship_id].active = false;
				pthread_mutex_unlock(&mutex_fleet);	
					
				pthread_mutex_lock(&mutex_route);
				routes[ship_id].trace_index = -1;
				pthread_mutex_unlock(&mutex_route);
				return step;
			}
		}
		else
		{
			follow_track_frw(ship_id, true);
		}

		if (exit_reached)
		{
			update_rr(ship_id, cur_repl, -1);
		}
	}
	return EGRESS;
}

void * ship_task(void * arg)
{
int ship_id;
enum state step = GUARD;
bool move;
bool c_end;
ship cur_ship;
triple mytrace[X_PORT * Y_PORT];
struct timespec now;

	// Task private variables
const int id = get_task_index(arg);
	set_activation(id);
	ship_id 			= id - AUX_THREAD;
	c_end				= false;

	while (!c_end) 
	{
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		pthread_mutex_lock(&mutex_fleet);
		cur_ship = fleet[ship_id];
		pthread_mutex_unlock(&mutex_fleet);
		
		if (cur_ship.active)
		{		
			switch (step)
			{
				case PORT:
					step = reach_port(ship_id, mytrace, cur_ship, true);
					break;

				case PLACE:
					step = reach_place(ship_id, mytrace, cur_ship, true);
					break;

				case EGRESS:
					step = reach_exit(ship_id, mytrace, cur_ship, true);
					break;

				default:
					move = (check_forward(cur_ship.x, cur_ship.y, 
											cur_ship.traj_grade)) ? false: true;
					step = reach_guard(ship_id, mytrace, cur_ship, move);

			}
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

void reverse_array(triple trace[X_PORT * Y_PORT], int last_index)
{
int i;
triple aux;
int size = last_index;
	for(i = 0; i < size / 2; ++i)
	{
		aux = trace[i];
		trace[i] = trace[last_index];
		trace[last_index] = aux;
		last_index --;
	}
}

void make_array_trace(BITMAP * t, triple trace[PORT_BMP_W * PORT_BMP_H], bool odd, int obj)
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
					trace[index] = make_triple(i,j, color);
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
					trace[index] = make_triple(i,j, color);
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

	for (j = (YSHIP/2) + 2; j < 130; ++j )
	{
		x = x_cur + j * cos(g_cur);
		y = y_cur + j * sin(g_cur) + (YSHIP / 2 );
		pthread_mutex_lock(&mutex_sea);
		color1 = getpixel(sea, x, y);
		pthread_mutex_unlock(&mutex_sea);

		if ((color1 != SEA_COLOR && color1 != -1))
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


void grade_filter(int id, int i, triple mytrace[X_PORT * Y_PORT])
{
float p = powf(M_E,(-0.115129 * PERIOD));
float grade = p * (degree_rect(fleet[id].x, fleet[id].y, mytrace[i].x,
						 mytrace[i].y)) + (1 - p) * (fleet[id].traj_grade);
fleet[id].traj_grade = grade;

}


void follow_track_frw(int id, bool move)
{
	float p = 0.02;
	float des;
	float acc;
	float d_obj;
	int red = makecol(255, 0, 0);
	int i, index, last_index, trace_index ;

	pthread_mutex_lock(&mutex_route);
	i = routes[id].index;
	last_index = routes[id].last_index;
	trace_index = routes[id].trace_index;
	pthread_mutex_unlock(&mutex_route);

	last_index = (move) ? last_index : i;
	
	pthread_mutex_lock(&mutex_fleet);
	d_obj = distance_vector(fleet[id].x, fleet[id].y, 
						array_routes[trace_index].enter_array[last_index].x, 
						array_routes[trace_index].enter_array[last_index].y);

	fleet[id].vel = MIN((d_obj) * p, fleet[id].vel);
	fleet[id].vel = MIN(MAX_VEL, fleet[id].vel);
	
	if (array_routes[trace_index].enter_array[i].color == red)
	{
		fleet[id].vel -= (fleet[id].vel - MIN_VEL) * p;
	}

	else 
	{
		fleet[id].vel += (MAX_VEL - fleet[id].vel) * p;
	}

	des = fleet[id].vel*PERIOD;
	acc = distance_vector(fleet[id].x, fleet[id].y, 
									array_routes[trace_index].enter_array[i].x, 
									array_routes[trace_index].enter_array[i].y);

	while (des > acc)
	{
		i += 1;
		acc += distance_vector(fleet[id].x, fleet[id].y, array_routes[trace_index].enter_array[i].x, 
																array_routes[trace_index].enter_array[i].y);
	}
	index = MIN(i, last_index);

	fleet[id].x = p * (array_routes[trace_index].enter_array[index].y) 
													+ (1 - p) * (fleet[id].x);

	fleet[id].y = p * (array_routes[trace_index].enter_array[index].y )
													+ (1 - p) * (fleet[id].y);

	if (move)
	{
		grade_filter(id, index, array_routes[trace_index].enter_array);
	}

	pthread_mutex_unlock(&mutex_fleet);

	pthread_mutex_lock(&mutex_route);
	routes[id].index = index;
	pthread_mutex_unlock(&mutex_route);
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
float aux = 1400 / PERIOD;
	if(x_cur < X_PORT)
	{
		if (x_cur > -YSHIP)
		{
			pthread_mutex_lock(&mutex_fleet);
			fleet[id].x -= fleet[id].vel / 2;//YSHIP / aux;
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

int find_index(triple mytrace[X_PORT * Y_PORT], int posix)
{
int i;
	for (i = 0; i < X_PORT * Y_PORT; ++i)
	{
		if (mytrace[i].y == posix)
			return i;
	}
}

void update_rr(int id, int repl, int req)
{
	pthread_mutex_lock(&mutex_rr);
	reply_access[id] = repl;
	request_access[id] = req;
	pthread_mutex_unlock(&mutex_rr);
}

bool get_repl(int ship_id)
{
bool repl;
	pthread_mutex_lock(&mutex_rr);
	repl = reply_access[ship_id];
	pthread_mutex_unlock(&mutex_rr);
	return repl;
}

bool get_req(int ship_id)
{
int req;
	pthread_mutex_lock(&mutex_rr);
	req = request_access[ship_id];
	pthread_mutex_unlock(&mutex_rr);
	return req;
}

int update_state(int id, int new_state, int obj, int last_index)
{
	update_rr(id, false, obj);

	pthread_mutex_lock(&mutex_route);
	if (last_index > 0)
	{
		routes[id].last_index = last_index;
	}
	pthread_mutex_unlock(&mutex_route);

	return new_state;
}

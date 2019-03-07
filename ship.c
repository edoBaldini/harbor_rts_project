#include "common.h"
#include "ship.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include "ptask.h"

//	Manages the behavior of a single ship from its ingress to its egress.
//	A task is identified by an id and a single task will manage a single ship.
void * ship_task(void * arg)
{
int ship_id;						//	id of a ship
enum state step = GUARD;			//	a ship starts with GUARD state
bool restrain;						//	says if the ship must brake
bool c_end = false;							//	if true the ship terminates
ship cur_ship;						//	contains the current values of the ship

//	array of triple containing the points that the ship must follow
triple mytrace[X_PORT * Y_PORT];
struct timespec now;				//	current hourly
const int id = get_task_index(arg);	//	id of the task
	
	set_activation(id);			// set the activation time and absolute deadline
	
	//	computes the id of the ship related to this task	
	ship_id 			= id - AUX_THREAD;

	while (!c_end)					// repeat until c_end is false
	{
	
		pthread_mutex_lock(&mutex_end);
		c_end = end;				//	update the current value of c_end
		pthread_mutex_unlock(&mutex_end);

		pthread_mutex_lock(&mutex_fleet);
		cur_ship = fleet[ship_id];	//	udate the current values of the ship
		pthread_mutex_unlock(&mutex_fleet);
		
		if (cur_ship.active)
		{
			switch (step)
			{
				//	ship is above YGUARD_POS and tries to reach Y_PORT
				case PORT:	
					step = reach_port(ship_id, mytrace, cur_ship, true);
					break;

				//	ship is above Y_PORT and tries to reach Y_PLACE
				case PLACE:
					step = reach_place(ship_id, mytrace, cur_ship, true);
					break;

				//	ship has terminated its parking time and tries to exit
				case EGRESS:
					step = reach_exit(ship_id, mytrace, cur_ship, true);
					break;

				//	ship is onset of the map and tries to reach YGUARD_POS
				default:

					// true if there is another ship ahead of this on the map
					restrain = check_forward(cur_ship.x, cur_ship.y, 
											cur_ship.traj_grade);
					step = reach_guard(ship_id, mytrace, cur_ship, restrain);

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
enum state reach_guard(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool restrain)
{
int index;
enum state step;
bool guard_reached = check_position(cur_ship.y, YGUARD_POS);
	pthread_mutex_lock(&mutex_route);
	index = routes[ship_id].index;
	pthread_mutex_unlock(&mutex_route);

	if (index == -1)
	{
		compute_mytrace(ship_id, mytrace, YGUARD_POS);
	}

	if(guard_reached)
	{
		index = find_index(mytrace, Y_PORT);
		pthread_mutex_lock(&mutex_route);
		routes[ship_id].last_index = index;
		pthread_mutex_unlock(&mutex_route);
		update_rr(ship_id, false, Y_PORT);

		return PORT;
	}

	follow_track_frw(ship_id, mytrace, restrain);
	return GUARD;
}

enum state reach_port(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool restrain)
{
enum state step;	
bool cur_repl = get_repl(ship_id);	
bool port_reached = check_position(cur_ship.y, Y_PORT);

	if (cur_repl)
	{
		if(port_reached)
		{
			update_rr(ship_id, false, Y_PLACE);
			return PLACE;
		}
		else 
		{
			follow_track_frw(ship_id, mytrace, false);
		}
	}

	return PORT;
}

enum state reach_place(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool restrain)
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
		if (index == -1)
		{
			compute_mytrace(ship_id, mytrace, Y_PLACE - YSHIP);
		}

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
					update_rr(ship_id, false, Y_EXIT);

					return EGRESS;
				}
			}
		}
		else
		{	
			follow_track_frw(ship_id, mytrace, false);
		}
	}
	return PLACE;	
}

enum state reach_exit(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool restrain)
{
int index;
enum state step;
bool cur_repl = get_repl(ship_id);
int cur_req = get_req(ship_id);
bool exit_reached = check_position(cur_ship.y, Y_PORT - XSHIP) && cur_req == Y_EXIT;
	
	pthread_mutex_lock(&mutex_route);
	index = routes[ship_id].index;
	pthread_mutex_unlock(&mutex_route);
	
	if (cur_repl)
	{ 
		if (index == -1)
		{
			compute_mytrace(ship_id, mytrace, Y_EXIT);
		}

		if (cur_ship.y < mytrace[0].y)
		{
			rotate90_ship(ship_id, cur_ship.x, Y_PLACE, mytrace[0].y + YSHIP);
		}
		else if (cur_ship.x <= EPSILON + YSHIP || cur_ship.x >= PORT_BMP_W - EPSILON - YSHIP)
		{
			exit_ship(ship_id, cur_ship.x);
			if (cur_ship.x < -YSHIP || cur_ship.x > PORT_BMP_W + YSHIP)
			{
				update_rr(ship_id, false, YGUARD_POS);
				cur_ship.active = false;

				pthread_mutex_lock(&mutex_fleet);
				fleet[ship_id].active = false;
				pthread_mutex_unlock(&mutex_fleet);	
					
				pthread_mutex_lock(&mutex_route);
				routes[ship_id].trace = NULL;
				pthread_mutex_unlock(&mutex_route);
				return GUARD;
			}
		}
		else
		{
			follow_track_frw(ship_id, mytrace, false);
		}

		if (exit_reached)
		{
			update_rr(ship_id, cur_repl, -1);
		}
	}
	return EGRESS;
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR SHIPS
//------------------------------------------------------------------------------
void compute_mytrace(int ship_id, triple mytrace[X_PORT * Y_PORT], int obj)
{
int index_objective;
bool is_flip;
BITMAP * cur_trace;

	pthread_mutex_lock(&mutex_route);
	cur_trace = routes[ship_id].trace;
	is_flip = routes[ship_id].flip;
	pthread_mutex_unlock(&mutex_route);

	make_array_trace(cur_trace, mytrace, is_flip, obj);
	index_objective =  find_index(mytrace, obj);
	
	pthread_mutex_lock(&mutex_route);
	routes[ship_id].index = 0;
	routes[ship_id].last_index = index_objective;
	pthread_mutex_unlock(&mutex_route);
}

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

void make_array_trace(BITMAP * t, triple trace[X_PORT * Y_PORT], bool flip, int obj)
{
int color;
int index = 0;
int i, j;
int last_index;
for (j = PORT_BMP_H; j > 0; --j)   
	{
		if (flip)
		{
			for (i = PORT_BMP_W; i > 0 ; --i)
			{
				color = getpixel(t, i, j);
				if (color == 0 || color == makecol(255, 0, 0))
				{
					trace[index] = make_triple(i, j , color);
					index++;
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
					trace[index] = make_triple(i, j, color);
					index++;
				}
			}
		}
	}
	last_index = --index;

	if (obj == Y_EXIT)
		reverse_array(trace, last_index);
}

bool check_forward(float x_cur, float y_cur, float g_cur)
{
float x, y, j;
int color;

	for (j = (YSHIP/2) + 2; j < 130; ++j )
	{
		x = x_cur + j * cos(g_cur);
		y = y_cur + j * sin(g_cur) + (YSHIP / 2 );
		pthread_mutex_lock(&mutex_sea);
		color = getpixel(sea, x, y);
		pthread_mutex_unlock(&mutex_sea);

		if ((color != SEA_COLOR && color != -1))
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
float grade_epsilon = 0.02;
float p = powf(M_E,(-0.115129 * PERIOD));
float grade = p * (degree_rect(fleet[id].x, fleet[id].y, mytrace[i].x,
						 mytrace[i].y)) + (1 - p) * (fleet[id].traj_grade);

	fleet[id].traj_grade = grade;

}


void follow_track_frw(int id, triple mytrace[X_PORT * Y_PORT], bool restrain)
{
	float p = 0.02;
	float des;
	float acc;
	float d_obj;
	int red = makecol(255, 0, 0);
	int i, index, last_index;

	pthread_mutex_lock(&mutex_route);
	i = routes[id].index;
	last_index = routes[id].last_index;
	pthread_mutex_unlock(&mutex_route);

	last_index = (restrain) ? i : last_index;
	
	pthread_mutex_lock(&mutex_fleet);
	d_obj = distance_vector(fleet[id].x, fleet[id].y, mytrace[last_index].x, 
												mytrace[last_index].y);
	fleet[id].vel = MIN((d_obj / 2) * p, fleet[id].vel);
	fleet[id].vel = MIN(MAX_VEL, fleet[id].vel);
	
	if (mytrace[i].color == red) {
		fleet[id].vel -= (fleet[id].vel - MIN_VEL) * p;
	} else {
		fleet[id].vel += (MAX_VEL - fleet[id].vel) * p;
	}

	des = fleet[id].vel * PERIOD;
	acc = distance_vector(fleet[id].x, fleet[id].y, mytrace[i].x, mytrace[i].y);

	while (des > acc)
	{
		i += 1;
		acc += distance_vector(fleet[id].x, fleet[id].y, mytrace[i].x, 
																mytrace[i].y);
	}
	index = MIN(i, last_index);

	fleet[id].x = p * (mytrace[index].x ) + (1 - p) * (fleet[id].x);
	fleet[id].y = p * (mytrace[index].y ) + (1 - p) * (fleet[id].y);

	if (!restrain)
	{
		grade_filter(id, index, mytrace);
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

int find_index(triple mytrace[X_PORT * Y_PORT], int y)
{
int i;
	for (i = 0; i < X_PORT * Y_PORT; ++i)
	{
		if (mytrace[i].y == y)
			return i;
	}
	return -1;
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

int get_req(int ship_id)
{
int req;
	pthread_mutex_lock(&mutex_rr);
	req = request_access[ship_id];
	pthread_mutex_unlock(&mutex_rr);
	return req;
}

triple make_triple(float x, float y, int color)
{
	triple coordinates;
	coordinates.x = x;
	coordinates.y = y;
	coordinates.color = color;
	return coordinates;
}

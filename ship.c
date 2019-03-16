#include "common.h"
#include "ship.h"
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include "ptask.h"

//------------------------------------------------------------------------------
//	SHIP GLOBAL CONSTANT
//------------------------------------------------------------------------------
#define MIN_P_TIME		360000		//	min ship parking time, in ms
#define	MAX_P_TIME		370000		//	max ship parking time, in ms
#define MIN_VEL			0.8			//	minimum speed
#define	MAX_VEL			1.5			//	maximum speed

//	Updates the attributes of the ship till it reaches its target.
enum state go_2_target(int ship_id, triple mytrace[X_PORT * Y_PORT], 
										ship cur_ship, bool curb, enum state s)
{
//	targets of the ship
int obj[5] = {YGUARD_POS, Y_PORT, Y_PLACE, Y_PARKED, Y_EXIT};

//	true when the ship will reach its current target
bool target_reached = check_position(cur_ship.y, obj[s]);	

	if(target_reached)
	{
		s += 1;		//	increase the state of the ship
		//	set safely request_access to next target and reply_access to false
		update_rr(ship_id, false, obj[s]);
		return s;
	}
	//	updates the ship attributes following the array mytrace
	follow_track_frw(ship_id, mytrace, curb);
	
	return s;	//	current state
}

//------------------------------------------------------------------------------
//	Updates the attributes of the ship till it reaches Y_PLACE.
//	When the ship reaches its place, a random parking time is set and the ship
//	will stay in its place till the parking time will be elapsed.
//------------------------------------------------------------------------------
enum state reach_place(int ship_id, triple mytrace[X_PORT * Y_PORT],
												ship cur_ship, bool curb)
{
enum state step = PLACE;
int parking_time;		//	period in which the ship will remain in the place

//	true when the bow reach the place's ingress
bool place_reached = check_position(cur_ship.y, Y_PLACE + YSHIP);
	
	//	updates the ship attributes till it reaches Y_PLACE
	step = go_2_target(ship_id, mytrace, cur_ship, curb, PLACE);

	if (place_reached) {
		//	set safely request_access to 1 means that the place is reached
		update_rr(ship_id, true, 1);
	}
		
	if (step == PARKED)	//	true only in the last execution of this function
	{
		//	if the ship is in place but is still not parked
		if (!cur_ship.parking) {	
			parking_time = random_in_range(MIN_P_TIME, MAX_P_TIME);
			cur_ship.parking = true;

			pthread_mutex_lock(&mutex_fleet);
			clock_gettime(CLOCK_MONOTONIC, &fleet[ship_id].p_time);
			time_add_ms(&fleet[ship_id].p_time, parking_time);
			fleet[ship_id].parking = true;
			pthread_mutex_unlock(&mutex_fleet);
		} 
	}
	return step;
}

//	When the parking time is elapsed, wake up the ship and request to EXIT
enum state wait_exit(int ship_id, ship cur_ship, enum state step)
{
int time_wakeup;		//	indicates when the ship has elapsed its parking time
int parking_time;		//	period in which the ship will remain in the place
struct timespec now;	//	current hourly

	clock_gettime(CLOCK_MONOTONIC, &now);
	time_wakeup = time_cmp(now, cur_ship.p_time);

	if (time_wakeup >= 0) {				//	if ship has elapsed its parking time

		pthread_mutex_lock(&mutex_fleet);
		fleet[ship_id].parking = false;
		pthread_mutex_unlock(&mutex_fleet);
		cur_ship.parking = false;

		//	set safely request_access to Y_EXIT and reply_access to false
		update_rr(ship_id, false, Y_EXIT);

		return step + 1;	//	next step EGRESS
	}
	return step;
}

//------------------------------------------------------------------------------
//	Updates the attributes of the ship till it goes outside of the map.
//	During the exiting the ship will follow three parts:
//	1) The ship will be drag down, to be aligned with the trace.
//	2) It will follow the trace and when will overcome the port will request -1.
//	3) At the end of the trace it will be drag out of the screen and it will be 
//	   deactivated.
//------------------------------------------------------------------------------
enum state reach_exit(int ship_id, triple mytrace[X_PORT * Y_PORT], 
													ship cur_ship, bool curb)
{	
enum state step = EGRESS;		
int cur_req = get_req(ship_id);	//	current request_access value
int x_point = Y_PORT - XSHIP;	//	point in which the ship is considered out

//	true when the ship has overcome the port and still requests Y_EXIT
bool exit_reached = check_position(cur_ship.y, x_point) && cur_req == Y_EXIT;
	
	//	if the ship is not aligned with the trace
	if (cur_ship.y < mytrace[0].y) {

		rotate90_ship(ship_id, cur_ship.x, Y_PLACE, mytrace[0].y);
	}

	//	if the ship is reaching the boundary of the map
	else if (cur_ship.x <= EPSILON + YSHIP || 
								cur_ship.x >= PORT_BMP_W - EPSILON - YSHIP)
	{
		//	updates the ship attributes guiding it outside the map
		exit_ship(ship_id, cur_ship.x);

		//	if the ship is outside of the map
		if (cur_ship.x < -YSHIP || cur_ship.x > PORT_BMP_W + YSHIP)
		{
		//	set safely request_access to YGUARD_POS and reply_access to false	
			update_rr(ship_id, false, YGUARD_POS);
			cur_ship.active = false;

			pthread_mutex_lock(&mutex_fleet);
			fleet[ship_id].active = false;	//	ship is no more active
			pthread_mutex_unlock(&mutex_fleet);	
					
			pthread_mutex_lock(&mutex_route);
			routes[ship_id].trace = NULL;
			pthread_mutex_unlock(&mutex_route);
			return GUARD;	//	come back to the first step
		}
	}
	else {

		//	updates the ship attributes following the array mytrace
		follow_track_frw(ship_id, mytrace, curb);
	}

	if (exit_reached) {
			
		//	set safely request_access to -1 means that the ship has left
		update_rr(ship_id, true, -1);
	}
	
	return step;
}

//	Set route's last_index of the given ship with array index relative to obj
void update_last_index(int s_id, triple mytrace[X_PORT * Y_PORT], int obj)
{
	//	computes the last index for the next step. 
	int last_index = find_index(mytrace, obj);	
	//	updates the route last index with the new value
	pthread_mutex_lock(&mutex_route);
	routes[s_id].last_index = last_index;	
	pthread_mutex_unlock(&mutex_route);	
}

//------------------------------------------------------------------------------
//	Given the coordinates x_cur, y_cur and the angle g_cur. It return true if 
//	detects a color different from SEA_COLOR between that position and a range.
//------------------------------------------------------------------------------
bool check_forward(float x_cur, float y_cur, float g_cur)
{
float x, y;			//	coordinates of the radius
float j;			//	length of the radius that is the range length
int color;			//	color detected over the map
int len_r = 80;		//	max point in which the ship can check

	for (j = (YSHIP/2) + 2; j < len_r; ++j )
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

//------------------------------------------------------------------------------
//	Given the ship id, the array of triples and if it must curb, it updates the
//	x, y coordinates of the ship and its inclination. To implement a smooth 
//	behaviour the ship attributes are updates in according to a low-pass filter
//------------------------------------------------------------------------------
void follow_track_frw(int id, triple mytrace[X_PORT * Y_PORT], bool curb)
{
	float p = 0.02;	//	low-pass filter parameter
	float des;		//	indicates how much the ship will move
	float acc;		//	accumulator of distances
	float d_obj;	//	indicates the distance between the ship and the target
	int red = makecol(255, 0, 0);
	int i, index, last_index;

	pthread_mutex_lock(&mutex_route);
	i = routes[id].index;	//	index of the position of the ship on the trace	
	last_index = routes[id].last_index;	//	indicates last index of the trace
	pthread_mutex_unlock(&mutex_route);
	
	//	if curb is true last_index = current index
	last_index = (curb) ? i : last_index;
	
	pthread_mutex_lock(&mutex_fleet);
	d_obj = distance_vector(fleet[id].x, fleet[id].y, mytrace[last_index].x, 
												mytrace[last_index].y);

	//	set the min velocity needed to reach the target
	fleet[id].vel = MIN((d_obj) * p, fleet[id].vel);
	fleet[id].vel = MIN(MAX_VEL, fleet[id].vel);
	
	if (mytrace[i].color == red) { 	//	if true slow down

		fleet[id].vel -= (fleet[id].vel - MIN_VEL) * p;
	} else {

		fleet[id].vel += p / 2;	//	constant acceleration factor
	}

	des = fleet[id].vel * PERIOD;
	acc = distance_vector(fleet[id].x, fleet[id].y, mytrace[i].x, mytrace[i].y);

	//	stop when the the sum of the distances between the points in the array
	//	reach the amount des
	while (des > acc){	

		i += 1;					//	new index of the array of triple
		acc += distance_vector(fleet[id].x, fleet[id].y, mytrace[i].x, 
																mytrace[i].y);
	}
	index = MIN(i, last_index);	//	check that i does not exceed the last index

	//	updates the coordinates of the ship considering the low-pass filter
	fleet[id].x = p * (mytrace[index].x ) + (1 - p) * (fleet[id].x);
	fleet[id].y = p * (mytrace[index].y ) + (1 - p) * (fleet[id].y);

	if (!curb)
	{
		//	update the inclination of the ship
		grade_filter(id, index, mytrace);
	}

	pthread_mutex_unlock(&mutex_fleet);

	//	updates the index of the position of the ship on the trace
	pthread_mutex_lock(&mutex_route);
	routes[id].index = index;
	pthread_mutex_unlock(&mutex_route);
}

//------------------------------------------------------------------------------
//	Given the ship id, the array of triple and its index, update the inclination
//	of the ship. The updating is affected by a low-pass filter.
//------------------------------------------------------------------------------
void grade_filter(int id, int i, triple mytrace[X_PORT * Y_PORT])
{
float p = 0.055;	//	low-pass filter factor
float grade = p * (degree_rect(fleet[id].x, fleet[id].y, mytrace[i].x,
						 mytrace[i].y)) + (1 - p) * (fleet[id].traj_grade);

	fleet[id].traj_grade = grade;

}

//	rates the angular coefficient (in rad) of the line passing between 2 points
float degree_rect(float x1, float y1, float x2, float y2)
{   
	float a_c = ((y2 - y1) / (x2 - x1));		//	angular coefficient in grade
	float degree = atanf(a_c);					//	angular coefficient in rad
	float offset = (x2 < x1) ? M_PI : 2 * M_PI;	//	needed to direct the bow
	
	return degree + offset;
}

//	Computes the euclidean distance of the provided points
float distance_vector (float x1, float y1, float x2, float y2)
{
float distance = sqrtf(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
	return distance;
}

//------------------------------------------------------------------------------
//	Given ship id and its target position, it fill mytrace calling 
//	make_array_trace and updates the variable last_index of ship' s route.
//------------------------------------------------------------------------------
void compute_mytrace(int ship_id, triple mytrace[X_PORT * Y_PORT], int obj)
{
int index_objective;	//	indicates the index of the target in the array
bool is_flip;			//	indicates if the trace must be flipped
int index;				//	index of the position of the ship on the trace
BITMAP * cur_trace;		//	current trace

	pthread_mutex_lock(&mutex_route);
	cur_trace = routes[ship_id].trace;
	is_flip = routes[ship_id].flip;
	index = routes[ship_id].index;
	pthread_mutex_unlock(&mutex_route);
	
	if (index == -1)			//	if a new trace is assigned, update mytrace
	{
		make_array_trace(cur_trace, mytrace, is_flip, obj);	//	fill mytrace
		index_objective =  find_index(mytrace, obj);

		pthread_mutex_lock(&mutex_route);
		routes[ship_id].index = 0;	//	ship' ll start from index 0 in the array
		routes[ship_id].last_index = index_objective;
		pthread_mutex_unlock(&mutex_route);
	}
}

//------------------------------------------------------------------------------
//	Given a bitmap, an array of triple, if the bitmap must be flipped and 
//	the target, it maps in the array of triple the bitmap.
//------------------------------------------------------------------------------
void make_array_trace(BITMAP * t, triple trace[X_PORT * Y_PORT], bool flip, 
																		int obj)
{

int color;			//	color found in a specific position on the bitmap
int index = 0;		//	index of the array
int i, j;			//	indexes to scan the bitamp
int last_index;		//	last index of the array
int red = makecol(255, 0, 0);

for (j = PORT_BMP_H; j > 0; --j)	//	scan from the botton of the bitmap
	{
		if (flip){					//	scan from the right
			for (i = PORT_BMP_W; i > 0 ; --i) {

				color = getpixel(t, i, j);
				if (color == 0 || color == red)
				{
					trace[index] = make_triple(i, j , color);	//	fill array
					index++;		// increment the array index
				}
			}
		}
		else {						//	scan from left
			for (i = 0; i < PORT_BMP_W; ++i) {

				color = getpixel(t, i, j);
				if (color == 0 || color == red)
				{
					trace[index] = make_triple(i, j, color);	//	fill array
					index++;		//	increment the array index
				}
			}
		}
	}
	last_index = --index;

	if (obj == Y_EXIT) {
	//	it is needed to reverse the array to move the ship from top to bottom
		reverse_array(trace, last_index);
	}
}

//	Given an array and its last index, updates the array inverting its elements.
void reverse_array(triple trace[X_PORT * Y_PORT], int last_index)
{
int i;
triple aux;					//	auxiliar triple
int size = last_index;		//	dimension of the array that must be reversed
	
	//	swap the element at position last_index with the element at position i
	for(i = 0; i < size / 2; ++i)
	{
		aux = trace[i];
		trace[i] = trace[last_index];
		trace[last_index] = aux;
		last_index --;
	}
}

//------------------------------------------------------------------------------
//	Returns the index in which the given y coordinate appears in the given array
//	of triple. Otherwise returns -1
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
//	Updates the position and inclination of the ship with the given id rotating
//	and drag it down from y1 to y2.
//------------------------------------------------------------------------------
void rotate90_ship(int id, float x_cur, int y1, int y2)
{
float aux = 10000 / PERIOD;			//	number of increments
float desired_degree = M_PI / 2;		//	final inclination of the ship
	
	if (x_cur > X_PORT)	//	the inclination will be decreased
	{
		pthread_mutex_lock(&mutex_fleet);
		fleet[id].y += (y2 - y1) / aux;
		fleet[id].traj_grade -= desired_degree / aux;
		pthread_mutex_unlock(&mutex_fleet);
	}
	else 				//	the inclination will be increased
	{
		pthread_mutex_lock(&mutex_fleet);
		fleet[id].y += (y2 - y1) / aux;
		fleet[id].traj_grade += (M_PI / 2) / aux;
		pthread_mutex_unlock(&mutex_fleet);
	}
}

//	Updates position of the ship with the given id dragging it out of the screen
bool exit_ship(int id, float x_cur)
{
	if(x_cur < X_PORT)			//	if the ship is to the left of the port
	{
		if (x_cur > -YSHIP)				//	if the ship is not  off the screen
		{
			pthread_mutex_lock(&mutex_fleet);
			fleet[id].x -= fleet[id].vel / 2;
			fleet[id].traj_grade = M_PI;
			pthread_mutex_unlock(&mutex_fleet);

			return false;
		}
		else return true;
	}
	else
	{	if (x_cur < PORT_BMP_W + YSHIP)	//	if the ship is not  off the screen
		{	
			pthread_mutex_lock(&mutex_fleet);
			fleet[id].x += fleet[id].vel / 2;
			fleet[id].traj_grade = 2 * M_PI;
			pthread_mutex_unlock(&mutex_fleet);
			return false;
		}
		else return true;
	}
}

//	Updates safely the request/reply values of the ship with specified id
void update_rr(int id, int repl, int req)
{
	pthread_mutex_lock(&mutex_rr);
	reply_access[id] = repl;
	request_access[id] = req;
	pthread_mutex_unlock(&mutex_rr);
}

//	Creates a triple with the specied fields
triple make_triple(float x, float y, int color)
{
	triple coordinates;
	coordinates.x = x;
	coordinates.y = y;
	coordinates.color = color;
	return coordinates;
}

//------------------------------------------------------------------------------
//	Manages the behavior of a single ship from its ingress to its egress, 
//	updating its position according with its routes and its velocity.
//	A task is identified by an id and a single task will manage a single ship.
//------------------------------------------------------------------------------
void * ship_task(void * arg)
{
//	array of the position that the ship must reach
int obj[5] = {YGUARD_POS, Y_PORT, Y_PLACE, Y_PARKED,Y_EXIT};
int s_id;						//	id of a ship
int last_index;
enum state step = GUARD;			//	a ship starts with GUARD state
bool curb;							//	says if the ship must brake
bool c_end = false;					//	if true the ship terminates
bool cur_repl;						//	current reply_access value
ship cur_ship;						//	contains the current values of the ship

//	array of triple containing the points that the ship must follow
triple mytrace[X_PORT * Y_PORT];
struct timespec now;				//	current hourly
const int id = get_task_index(arg);	//	id of the task
	
	set_activation(id);			// set the activation time and absolute deadline
	
	
	s_id = id - AUX_THREAD;	//	computes the ship id that a task will manage

	while (!c_end)						// repeat until c_end is false
	{
		pthread_mutex_lock(&mutex_end);
		c_end = end;					//	update the current value of c_end
		pthread_mutex_unlock(&mutex_end);

		pthread_mutex_lock(&mutex_fleet);
		cur_ship = fleet[s_id];		//	udate the current values of the ship
		pthread_mutex_unlock(&mutex_fleet);

		cur_repl = get_repl(s_id);	
		
		if (cur_ship.active)
		{

			compute_mytrace(s_id, mytrace, obj[step]);

			switch (step)
			{
				//	ship is above YGUARD_POS and tries to reach Y_PORT
				case PORT:	
					if (cur_repl)
						step = go_2_target(s_id, mytrace, cur_ship, curb, PORT);
					break;

				//	ship is above Y_PORT and tries to reach Y_PLACE
				case PLACE:
					if (cur_repl)
						step = reach_place(s_id, mytrace, cur_ship, false);
					break;

				//	ship is parked and waits for exit
				case PARKED:
					step = wait_exit(s_id, cur_ship, PARKED);
					break;

				//	ship has terminated its parking time and tries to exit
				case EGRESS:
					if (cur_repl)
						step = reach_exit(s_id, mytrace, cur_ship, false);
					break;

				//	ship is onset of the map and tries to reach YGUARD_POS
				default:				
					//	true if there is another ship ahead of this on the map
					//	this is the unique case in which curb can be true
					curb = check_forward(cur_ship.x, cur_ship.y, 
											cur_ship.traj_grade);

					step = go_2_target(s_id, mytrace, cur_ship, curb, GUARD);

					if (step == PORT)	//	true when ship has reached Y_PORT
					{
						//	since trace will not change must change last_index
						update_last_index(s_id, mytrace, obj[step]);
					}
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

#ifndef SHIP_H
#define SHIP_H

//------------------------------------------------------------------------------
//	SHIP FUNCTIONS:	manage the behaviour of a ship from ingress to egress.
//
//	A ship will follow the traces loaded in its route. Each trace will be turned
//	in an array of positions. Depending on the velocity and the distance to the 
//	target the ship will assume a position in that array till reach the target.
//
//	It will follow in total 3 traces:
//	- first, to reach the access port position and in that position it assume
//		the PLACE state
//	- second, to reach the assigned place, and it will assume the EGRESS state
//	- third, to exit.
//	
//	A ship starts in GUARD state where its velocity depends on the detection
//	of others ships ahead in the sea to avoid clashes. Once reached the 
//	YGUARD_POS the ships will move one at a time, assuming PORT state, and the 
//	detection ability will be no longer necessary.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	States that a ship must follow to enter and exit the port.
//
//	GUARD: 	all the ships are in GUARD at the beginning and till they reach  
//			YGUARD_POS. Then the ships can request for the next state i.e. PORT.
//
//	PORT:	one ship is in PORT when its request has been approved and till it
//			reaches Y_PORT. Then the ship can request the next state i.e. PLACE.
//
//	PLACE:	one ship is in PLACE when its request has been approved and till
//			it's parking time doesn't elapsed. Then it can request for EGRESS.
//
//	EGRESS:	one ship is in EGRESS when its request has been approved and till it
//			exceed the entrance to the port.
//
//	More ships at the same time can be in GUARD state.
//	In the others state it is possible to have only one ship at a time.
//------------------------------------------------------------------------------
enum state {GUARD, PORT, PLACE, PARKED, EGRESS};

typedef struct triple 			//	struct used to build the array of positions
{
	float x, y;					//	coordinates
	int color;					//	color of a bitmap in that coordinates
}triple;

//------------------------------------------------------------------------------
//	SHIP FUNCTIONS
//------------------------------------------------------------------------------

//	Updates the attributes of the ship till it reaches its target.
enum state go_2_target(int ship_id, triple mytrace[X_PORT * Y_PORT], 
						ship cur_ship, bool curb, enum state s);

//	Updates the attributes of the ship till it reaches Y_PLACE.
enum state reach_place(int ship_id, triple mytrace[X_PORT * Y_PORT], 
													ship cur_ship, bool curb);

//	When the parking time is elapsed, wake up the ship and request to EXIT
enum state wait_exit(int ship_id, ship cur_ship, enum state step);

//	Updates the attributes of the ship till it goes outside of the map
enum state reach_exit(int ship_id, triple mytrace[X_PORT * Y_PORT], 
													ship cur_ship, bool curb);

//	Set route's last_index of the given ship with array index relative to obj
void update_last_index(int s_id, triple mytrace[X_PORT * Y_PORT], int obj);

//------------------------------------------------------------------------------
//	Given the coordinates x_cur, y_cur and the angles g_cur,
//	It return true if detects something between that position and a fixed range.
//------------------------------------------------------------------------------
bool check_forward(float x_cur, float y_cur, float g_cur);

//------------------------------------------------------------------------------
//	Given the ship id, the array of triples and if it must curb, it updates the
//	x, y coordinates of the ship and its inclination.
//------------------------------------------------------------------------------
void follow_track_frw(int id, triple mytrace[], bool curb);

//------------------------------------------------------------------------------
//	Given the ship id, the array of triple and its index, update the inclination
//	of the ship
//------------------------------------------------------------------------------
void grade_filter(int id, int i, triple mytrace[X_PORT * Y_PORT]);

//	rates the angular coefficient (in rad) of the line passing between 2 points
float degree_rect(float x1, float y1, float x2, float y2);

//	Computes the euclidean distance of the provided points
float distance_vector (float x1, float y1, float x2, float y2);

//------------------------------------------------------------------------------
//	Given ship id and its target position, it fill mytrace calling 
//	make_array_trace and updates the variable last_index of ship' s route.
//------------------------------------------------------------------------------
void compute_mytrace(int ship_id, triple mytrace[X_PORT * Y_PORT], int obj);

//------------------------------------------------------------------------------
//	Given a bitmap, an array of triple, if the bitmap must be flipped and 
//	the target, it maps in the array of triple the bitmap.
//------------------------------------------------------------------------------
void make_array_trace(BITMAP * t, triple trace[], bool flip, int req);

//------------------------------------------------------------------------------
//	Given an array and its last index, updates the array inverting its elements.
//	(e.g. the first element will be the last)
//------------------------------------------------------------------------------
void reverse_array(triple trace[], int last_index);

//------------------------------------------------------------------------------
//	Returns the index in which the given y coordinate appears in the given array
//	of triple. Otherwise returns -1.
//------------------------------------------------------------------------------
int find_index(triple mytrace[X_PORT * Y_PORT], int y);

//------------------------------------------------------------------------------
//	Updates the position and inclination of the ship with the given id rotating
//	and drag it down from y1 to y2.
//------------------------------------------------------------------------------
void rotate90_ship(int id, float x_cur, int y1, int y2);

//	Updates position of the ship with the given id dragging it out of the screen
bool exit_ship(int id, float x_cur);

//	Updates safely the request/reply values of the ship with specified id. 
void update_rr(int id, int repl, int req);

//	Creates a triple with the specied fields
triple make_triple(float x, float y, int color);

//	Manages the behavior of a single ship from its ingress to its egress
void * ship_task(void * arg);

#endif
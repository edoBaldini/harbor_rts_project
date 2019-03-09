#ifndef SHIP_H
#define SHIP_H

//------------------------------------------------------------------------------
//	States that a ship must follow to enter and exit the port.
//
//	GUARD: 	all the ships are in GUARD at the beginning and till they reaches  
//			YGUARD_POS. Then the ships can request for the next state i.e. PORT.
//
//	PORT:	one ship is in PORT when its request has been approved and till it
//			reaches YPORT. Then the ship can request the next state i.e. PLACE.
//
//	PLACE:	one ship is in PLACE when its request has been approved and till
//			it's parking time doesn't elapsed. Then it can request for the
//			next step i.e. EGRESS.
//
//	EGRESS:	one ship is in EGRESS when its request has been approved and till it
//			exceed the entrance to the port.
//
//	More ships at the same time can be in GUARD state.
//	In the others state it is possible to have only one ship at a time.
//------------------------------------------------------------------------------

typedef struct triple 			//	struct used to build the array of positions
{
	float x, y;					//	coordinates
	int color;					//	color of a bitmap in that coordinates
}triple;

enum state {GUARD, PORT, PLACE, EGRESS};

//------------------------------------------------------------------------------
//	SHIP FUNCTIONS
//------------------------------------------------------------------------------

//	Manages the behavior of a single ship from its ingress to its egress
void * ship_task(void * arg);

//	Defines the behavior of a ship that has to reach the guard position
enum state reach_guard(int ship_id, triple mytrace[X_PORT * Y_PORT], 
													ship cur_ship, bool curb);

//	Defines the behavior of a ship that has to reach the port position
enum state reach_port(int ship_id, triple mytrace[X_PORT * Y_PORT], 
													ship cur_ship, bool curb);

//	Defines the behavior of a ship that has to reach the place position
enum state reach_place(int ship_id, triple mytrace[X_PORT * Y_PORT], 
													ship cur_ship, bool curb);

//	Defines the behavior of a ship that has to reach the exit
enum state reach_exit(int ship_id, triple mytrace[X_PORT * Y_PORT], 
													ship cur_ship, bool curb);

//	Given the coordinates x_cur, y_cur and the angles g_cur,
//	It return true if detects something between that position and a fixed range.
bool check_forward(float x_cur, float y_cur, float g_cur);

//	Given the ship id, the array of triples and if it must curb, it updates the
//	x, y coordinates of the ship and its inclination.
void follow_track_frw(int id, triple mytrace[], bool curb);

//	Given the ship id, the array of triple and its index, update the inclination
//	of the ship
void grade_filter(int id, int i, triple mytrace[X_PORT * Y_PORT]);

//	rates the angular coefficient (in rad) of the line passing between 2 points
float degree_rect(float x1, float y1, float x2, float y2);

//	Computes the euclidean distance of the provided points
float distance_vector (float x1, float y1, float x2, float y2);

//	Given ship id and its target position, it fill mytrace calling 
//	make_array_trace and updates the variable last_index of ship' s route.
void compute_mytrace(int ship_id, triple mytrace[X_PORT * Y_PORT], int obj);

//	Given a bitmap, an array of triple, if the bitmap must be flipped and 
//	the target, it maps in the array of triple the bitmap.
void make_array_trace(BITMAP * t, triple trace[], bool flip, int req);

//	Given an array and its last index, updates the array inverting its elements.
//	(e.g. the first element will be the last)
void reverse_array(triple trace[], int last_index);

//	Returns the index in which the given y coordinate appears in the given array
//	of triple. Otherwise returns -1.
int find_index(triple mytrace[X_PORT * Y_PORT], int y);

//	Updates the position and inclination of the ship with the given id rotating
//	and drag it down from y1 to y2.
void rotate90_ship(int id, float x_cur, int y1, int y2);

//	Updates position of the ship with the given id dragging it out of the screen
bool exit_ship(int id, float x_cur);

//	Updates safely the request/reply values of the ship with specified id. 
void update_rr(int id, int repl, int req);

//	Get reply_access value associated to the given ship_id safely
bool get_repl(int ship_id);

//	Get request_access value associated to the given ship_id safely
int get_req(int ship_id);

//	Creates a triple with the specied fields
triple make_triple(float x, float y, int color);

#endif
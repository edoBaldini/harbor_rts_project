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
enum state {GUARD, PORT, PLACE, EGRESS};

//------------------------------------------------------------------------------
//	SHIP FUNCTIONS
//------------------------------------------------------------------------------

//	Given the coordinates x_cur, y_cur and the angles g_cur,
//	It return true if detects something between that position and a fixed range.
bool check_forward(float x_cur, float y_cur, float g_cur);

//	Given the ship id, the array of the positions and the permission to move, it
//	updates the x, y coordinates of the ship and its inclination.
void follow_track_frw(int id, triple mytrace[], bool move);

//	Given ship id and its target position, it fill mytrace calling 
//	make_array_trace and updates the last_index ship route with the 
//	index with which to reach the target position.
void compute_mytrace(int ship_id, triple mytrace[X_PORT * Y_PORT], int obj);

//	Given a bitmap, an array of triple, if the bitmap must be flipped and 
//	the target, it maps in the array the points and the colors of the drawn
//	contained in the bitmap. 
void make_array_trace(BITMAP * t, triple trace[], bool flip, int req);

//	Given an array and its last index, updates the array inverting its elements.
//	(e.g. the first element will be the last)
void reverse_array(triple trace[], int last_index);

//	Given the ship id, its x current position, starting position y1 and target 
//	position y2, updates the position and inclination of the ship rotating it 
//	and drag it down from y1 to y2
void rotate90_ship(int id, float x_cur, int y1, int y2);

//	Given the ship id and its x current position, updates its position dragging 
//	it out of the screen
bool exit_ship(int id, float x_cur);


float distance_vector (float x1, float y1, float x2, float y2);
int find_index(triple mytrace[X_PORT * Y_PORT], int posix);
void update_rr(int id, int repl, int req);
bool get_repl(int ship_id);
bool get_req(int ship_id);
int update_state(int id, int new_state, int obj, int last_index);
void * ship_task(void * arg);

enum state reach_guard(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool move);
enum state reach_port(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool move);
enum state reach_place(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool move);
enum state reach_exit(int ship_id, triple mytrace[X_PORT * Y_PORT], ship cur_ship, bool move);

#endif
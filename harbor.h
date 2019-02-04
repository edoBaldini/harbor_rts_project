#ifndef HARBOR_H
#define HARBOR_H
#include <allegro.h>

//-----------------------------------------------------------------------------
// GLOBAL STRUCTURE
//------------------------------------------------------------------------------
typedef int bool;

typedef struct ship 
{
	float x, y;
	float traj_grade; 
	BITMAP * boat;
	struct timespec p_time;
	bool parking;
	bool active;
}ship;

typedef struct pair
{
	float x, y;
}pair;

typedef struct route
{
	BITMAP * trace;
	bool odd;
	float x, y; 
}route;

typedef struct place 
{
	BITMAP * enter_trace;
	BITMAP * exit_trace;
	int ship_id;
	bool available;
}place; 

//------------------------------------------------------------------------------
//	RADAR FUNCTIONS
//------------------------------------------------------------------------------
bool check_ship(int j, int color);
float degree_rect(float x1, float y1, float x2, float y2);
float degree_fix(float grade);

//------------------------------------------------------------------------------
//	SHIP FUNCTIONS
//------------------------------------------------------------------------------
void reverse_array(pair trace[], int last_index);
int make_array_trace(BITMAP * t, pair trace[], int id, bool odd);
bool check_forward(float x_cur, float y_cur, float g_cur);
bool check_yposition(float y_ship, int y);
void follow_track_frw(int id, int i, pair mytrace[], int last_index);
void rotate90_ship(int id, int y1, int y2);
bool exit_ship(int id);

//------------------------------------------------------------------------------
//	USER FUNCTIONS
//------------------------------------------------------------------------------
int click_place(int offset, int delta, int l_x, int r_x);

//------------------------------------------------------------------------------
//	CONTROLLER FUNCTIONS
//------------------------------------------------------------------------------
bool try_access_port();
bool assign_trace(int ship);
bool assign_exit(int ship);
void free_trace(int ship);

//------------------------------------------------------------------------------
//	AUXILIAR FUNCTIONS
//------------------------------------------------------------------------------
void init(void);
void init_ship();
void fill_places();
void mark_label(BITMAP * boat);
void terminate();
int random_in_range(int min_x, int max_x);
pair make_pair(int x, int y);

#endif

#ifndef HARBOR_H
#define HARBOR_H
#include <allegro.h>

#define XWIN			1400						// width monitor
#define YWIN			900							// height monitor
#define PERIOD			25							// in ms
#define DLINE			20							// in ms
#define PRIO			10							// priority level
#define AUX_THREAD 		4							// # of auxiliar threads
#define MAX_THREADS		20							// total number of threads
#define MAX_SHIPS		MAX_THREADS - AUX_THREAD	// number of ships
#define EPSILON			3.f							// guardian distance to goal
#define N_INGRESS		3							// number of entering tracks
#define	N_PLACE	8							// number of parking
#define	SEA_COLOR 		makecol(0,85,165) 			// background color
//------------------------------------------------------------------------------
//	FOR BOOLEAN DEFINITION
//------------------------------------------------------------------------------
#define true			1
#define false			0

//------------------------------------------------------------------------------
//	SHIP GLOBAL CONSTANT
//------------------------------------------------------------------------------
#define XSHIP			18		// width of the ship  
#define YSHIP			54		// height of the ship
#define MIN_P_TIME		20000	// min ship parking time, in ms
#define	MAX_P_TIME		70000	// max ship parking time, in ms
#define MIN_VEL			1		// minimum speed
#define	MAX_VEL			3		// maximum speed

//-----------------------------------------------------------------------------
// GLOBAL CONSTANTS related to the radar
//------------------------------------------------------------------------------
#define R_BMP_W			451		//	width of the radar
#define R_BMP_H			451		//	height of the radar

#define RMAX			450		//	max radius 
#define ARES			360		// 	max alpha
#define RSTEP			1		//	step of the radius
#define RMIN			0		//	min radius
#define XRAD			450		//	xcenter of the radar = center of the port
#define YRAD			450		//	y center of the radar = center of the port

//------------------------------------------------------------------------------
//	POSITION GLOBAL CONSTANTS
//------------------------------------------------------------------------------
#define YGUARD_POS		610		//	y position where the ships wait
#define X_PORT			450		//	x position of the door port
#define Y_PORT			505		//	y postizion of the door port

#define PORT_BMP_W		900		//	width port 
#define PORT_BMP_H		900		//	height port
#define	Y_PLACE			253		//	y value of the places
#define Y_EXIT			490		//	y value of the port exit

//-----------------------------------------------------------------------------
// GLOBAL STRUCTURE
//------------------------------------------------------------------------------
typedef int bool;

typedef struct ship 
{
	float x, y;					// position of the ship
	float traj_grade; 			// angle of inclination, in rad
	float vel;					// speed of the ship 
	struct timespec p_time;		// parking time
	bool parking;				// if the ship is parked
	bool active;				// if the ship is active
}ship;

typedef struct triple 			// triple exploited to build the positions array
{
	float x, y;				
	int color;				
}triple;

typedef struct route 			// struct related to ship identifying its route
{
	int trace_index;			// path that the ships will follow
	int index;					// index of the ship in the positions array
	int last_index;				// last index of the positions array
}route;

typedef struct place 			// structure that identify a parking spot
{
	BITMAP * enter_trace;		// trace that must be followed to reach the spot
	BITMAP * exit_trace;		// trace that must be followed to  exit
	int ship_id;				// id of the ship associated to a place
	bool available;				// if a place is free or not
}place; 

typedef struct track 			// structure that identify a parking spot
{
	BITMAP * enter_trace;		// trace that must be followed to reach the spot
	BITMAP * exit_trace;		// trace that must be followed to  exit
	int ship_id;				// id of the ship associated to a place
	bool available;				// if a place is free or not
}track; 

typedef struct array_route
{
	triple enter_array[PORT_BMP_W * PORT_BMP_H];
	triple exit_array[PORT_BMP_W * PORT_BMP_H];
}array_route;

//-----------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
extern BITMAP * sea;
extern BITMAP * enter_trace[3];

extern struct place places[N_PLACE];

extern struct track tracks[N_PLACE + N_INGRESS];

extern struct ship fleet[MAX_SHIPS];
extern struct route routes[MAX_SHIPS];

extern struct array_route array_routes[N_PLACE + N_INGRESS];

extern int ships_activated;
extern int request_access[MAX_SHIPS];

extern bool reply_access[MAX_SHIPS];
extern bool end;
extern bool show_routes;
extern pthread_mutex_t mutex_fleet;
extern pthread_mutex_t mutex_route;
extern pthread_mutex_t mutex_rr;
extern pthread_mutex_t mutex_p;
extern pthread_mutex_t mutex_sea;
extern pthread_mutex_t mutex_end;
extern pthread_mutex_t mutex_s_route;

//------------------------------------------------------------------------------
//	COMMON FUNCTIONS
//------------------------------------------------------------------------------
float degree_rect(float x1, float y1, float x2, float y2);
int random_in_range(int min_x, int max_x);
triple make_triple(float x, float y, int color);
bool check_position(float y_ship, int y);
void * user_task(void * arg);
void * ship_task(void * arg);
void reverse_array(triple trace[], int last_index);
void make_array_trace(BITMAP * t, triple trace[], bool odd, int req);
int find_index(triple mytrace[X_PORT * Y_PORT], int posix);

#endif

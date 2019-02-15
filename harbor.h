#ifndef HARBOR_H
#define HARBOR_H
#include <allegro.h>

#define XWIN			1400						// width monitor
#define YWIN			700							// height monitor
#define PERIOD			20							// period in ms
#define DLINE			15							// deadline in ms
#define PRIO			10							// priority level
#define AUX_THREAD 		4							// auxiliar thread
#define MAX_THREADS		15							// max number of threads	
#define MAX_SHIPS		MAX_THREADS - AUX_THREAD	// max number of ships
#define FPS				200.0
#define FRAME_PERIOD	(1 / FPS)
#define EPSILON			3.f							// guardian distance to goal
#define	PLACE_NUMBER	8							// number of parking
#define	SEA_COLOR 		makecol(0,85,165)			// sea color in rgb
//------------------------------------------------------------------------------
//	FOR BOOLEAN DEFINITION
//------------------------------------------------------------------------------
#define true			1
#define false			0

//------------------------------------------------------------------------------
//	SHIP GLOBAL CONSTANT
//------------------------------------------------------------------------------
#define XSHIP			18						// width dimension of the ship  
#define YSHIP			54						// height dimension of the ship
#define MIN_P_TIME		100						// min ship parking time, in ms
#define	MAX_P_TIME		30000					// max ship parking time, in ms
#define MIN_VEL			0.5						// minimum speed
#define	MAX_VEL			3						// maximum speed

//-----------------------------------------------------------------------------
// GLOBAL CONSTANTS related to the radar
//------------------------------------------------------------------------------
#define R_BMP_W			451						// width of the radar
#define R_BMP_H			451						// height of the radar

#define RMAX			450						// max radius 
#define ARES			360						//  max alpha
#define RSTEP			1						// step of the radius
#define RMIN			0						// min radius
#define XRAD			450						// x center of the radar
#define YRAD			450						// y center of the radar

//------------------------------------------------------------------------------
//	POSITION GLOBAL CONSTANTS
//------------------------------------------------------------------------------
#define YGUARD_POS		610					// y position where the ships wait
#define X_PORT			450					// x position of the door port
#define Y_PORT			505					// y postizion of the door port

#define PORT_BMP_W		900					// width port 
#define PORT_BMP_H		900					// height port
#define	Y_PLACE			253					// y value of the places
#define Y_EXIT			330					// y value of the port exit
//------------------------------------------------------------------------------
// GLOBAL STRUCTURE
//------------------------------------------------------------------------------
typedef int bool;

typedef struct ship 
{
	float x, y;								// x, y coordinates
	float traj_grade; 						// ship angle
	float vel;								// ship velocity
	BITMAP * boat;							// ship bitmap
	struct timespec p_time;					// ship parking time
	bool parking;							// parking status
	bool active;							// active status
}ship;

typedef struct pair
{
	float x, y;
}pair;

typedef struct route
{
	BITMAP * trace;							// route bitmap
	bool odd;								// check if it is left route
	float x, y; 							// last coordinates of the route
}route;

typedef struct place 
{
	BITMAP * enter_trace;					// entertrace bitmap
	BITMAP * exit_trace;					// exit trace bitap
	int ship_id;							// ships id to which the place has
											// been assigned
	bool available;							// availability status
}place; 

extern BITMAP * sea;						// sea bitmap

extern struct ship fleet[MAX_SHIPS];		// array that maintains all ships
extern struct route routes[MAX_SHIPS];

extern int request_access[MAX_SHIPS];		// array that maintains the requests
											// 	of all the ships
extern bool reply_access[MAX_SHIPS];		// array that maintains the replies
											//	of all the ships
extern bool end;							// variable that identifies the 
											//	execution end
extern pthread_mutex_t mutex_fleet;			// mutex variable for the fleet
extern pthread_mutex_t mutex_route;			// mutex variable for the routes
extern pthread_mutex_t mutex_rr;			// mutex variable for request/reply
extern pthread_mutex_t mutex_sea;			// mutex variable for the sea
extern pthread_mutex_t mutex_end;			// mutex variable for the end

//------------------------------------------------------------------------------
//	SHIP FUNCTIONS
//------------------------------------------------------------------------------
void * ship_task(void * arg);
void reverse_array(pair trace[], int last_index);
int make_array_trace(BITMAP * t, pair trace[], int id, bool odd, int req);
bool check_forward(float x_cur, float y_cur, float g_cur);
bool check_position(float y_ship, int y);
int follow_track_frw(int id, int i, pair mytrace[], int last_index, bool move, 
															BITMAP * cur_trace);
void rotate90_ship(int id, float x_cur, int y1, int y2);
bool exit_ship(int id, float x_cur);
float distance_vector (float x1, float y1, float x2, float y2);
int find_index(pair mytrace[X_PORT * Y_PORT], int posix);

#endif

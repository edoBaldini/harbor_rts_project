#ifndef HARBOR_H
#define HARBOR_H
#include <allegro.h>

#define XWIN			1400					//	width monitor
#define YWIN			700						//	height monitor
#define PERIOD			25						//	in ms
#define DLINE			20						//	in ms
#define PRIO			10						//	priority level
#define AUX_THREAD 		4						//	# of auxiliar threads
#define MAX_THREADS		20						//	total number of threads
#define MAX_SHIPS		MAX_THREADS - AUX_THREAD//	number of ships
#define EPSILON			3.f						//	guardian distance to goal
#define ENTER_NUMBER	3						//	number of entering tracks
#define	PLACE_NUMBER	8						//	number of parking
#define	SEA_COLOR 		makecol(0,85,165) 		//	background color
//------------------------------------------------------------------------------
//	FOR BOOLEAN DEFINITION
//------------------------------------------------------------------------------
#define true			1
#define false			0

//------------------------------------------------------------------------------
//	SHIP GLOBAL CONSTANT
//------------------------------------------------------------------------------
#define XSHIP			18			//	width ship  
#define YSHIP			54			//	height ship
#define MIN_P_TIME		20000		//	min ship parking time, in ms
#define	MAX_P_TIME		70000		//	max ship parking time, in ms
#define MIN_VEL			1			//	minimum speed
#define	MAX_VEL			3			//	maximum speed

//-----------------------------------------------------------------------------
// GLOBAL CONSTANTS related to the radar
//------------------------------------------------------------------------------
#define R_BMP_W			451			//	width radar
#define R_BMP_H			451			//	height radar

#define RMAX			450			//	max radius 
#define ARES			360			// 	max alpha
#define RSTEP			1			//	step of the radius
#define RMIN			0			//	min radius
#define XRAD			450			//	x radar center = x port center
#define YRAD			450			//	y radar center = y port center

//------------------------------------------------------------------------------
//	POSITION GLOBAL CONSTANTS
//------------------------------------------------------------------------------
#define YGUARD_POS		610			//	y position where the ships wait
#define X_PORT			450			//	x position of the door port
#define Y_PORT			505			//	y postizion of the door port

#define PORT_BMP_W		900			//	width port 
#define PORT_BMP_H		900			//	height port
#define	Y_PLACE			253			//	y value of the places
#define Y_EXIT			490			//	y value of the port exit

//-----------------------------------------------------------------------------
// GLOBAL STRUCTURE
//------------------------------------------------------------------------------
typedef int bool;

typedef struct ship 				
{
	float x, y;					//	ship coordinates
	float traj_grade; 			//	inclination angle
	float vel;					//	speed
	struct timespec p_time;		//	parking time
	bool parking;				//	if ship is parked
	bool active;				//	if ship is active 
}ship;

typedef struct triple 			//	struct used to build the positions array
{
	float x, y;					//	coordinates
	int color;					//	color of a bitmap in that coordinates
}triple;

typedef struct route 			// struct related to ship identifying its route
{
	BITMAP * trace;				// trace that the ship must follow
	bool flip;					// if the trace must be flipped
	int index;					// position along the trace
	int last_index;				// last position
}route;

typedef struct place 			// structure that identify a parking spot
{
	BITMAP * enter_trace;		
	BITMAP * exit_trace;
	int ship_id;				// ship assigned to a specific place
	bool available;				// true when a place is not assigned to a ship
}place; 

//-----------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
extern BITMAP * sea;						// bitmap in which are drawn ships			
extern BITMAP * enter_trace[3];				// array of the ingress trace bitmap

extern struct place places[PLACE_NUMBER];	// places used by controller & user
extern struct ship fleet[MAX_SHIPS];		// fleet used by display, ship, user
extern struct route routes[MAX_SHIPS];		// routes used by display, user ship
											//			 			& controller

extern int ships_activated;	
extern int request_access[MAX_SHIPS];		// positions required by ships

extern bool reply_access[MAX_SHIPS];		// ship allowed to move or not
extern bool end;							// true when user presses ESC
extern bool show_routes;					// true when user presses SPACE BAR 

// mutex for access to the global variables-------------------------------------
extern pthread_mutex_t mutex_fleet;			
extern pthread_mutex_t mutex_route;
extern pthread_mutex_t mutex_rr;			// for request_acces & reply_access
extern pthread_mutex_t mutex_p;				// for places
extern pthread_mutex_t mutex_sea;
extern pthread_mutex_t mutex_end;
extern pthread_mutex_t mutex_s_route;		// for show_route


//------------------------------------------------------------------------------
//	COMMON FUNCTIONS
//------------------------------------------------------------------------------

//	calculate the angular coefficient (in rad) of the line 
//	passing between the 2 points
float degree_rect(float x1, float y1, float x2, float y2);

//	calulate a random integer in the specified interval
int random_in_range(int min_x, int max_x);

//	creates a triple with the specied fields
triple make_triple(float x, float y, int color);

//	check if the two position differs for a tolerant space epsilon
bool check_position(float y_ship, int y);

void * user_task(void * arg);
void * ship_task(void * arg);

#endif

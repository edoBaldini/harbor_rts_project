#ifndef HARBOR_H
#define HARBOR_H
#include <allegro.h>

//------------------------------------------------------------------------------
//	COMMON:	contains the constants, the data structures, the variables and the
//	functions that are in common with others file.
//------------------------------------------------------------------------------

#define XWIN			1400					//	width monitor
#define YWIN			700						//	height monitor
#define PERIOD			25						//	in ms
#define DLINE			20						//	in ms
#define PRIO			10						//	priority level
#define AUX_THREAD 		4						//	# of auxiliar threads
#define MAX_THREADS		19						//	total number of threads
#define MAX_SHIPS		MAX_THREADS - AUX_THREAD	//	number of ships
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
#define XSHIP			16			//	width ship  
#define YSHIP			52			//	height ship

//------------------------------------------------------------------------------
//	POSITION GLOBAL CONSTANTS
//------------------------------------------------------------------------------
#define YGUARD_POS		610			//	y position where the ships wait
#define X_PORT			450			//	x position of the door port
#define Y_PORT			505			//	y position of the door port
#define	Y_PLACE			201			//	y value of the places
#define	Y_PARKED		Y_PLACE - 1	//	y value of a parked ship
#define Y_EXIT			490			//	y value of the port exit

#define PORT_BMP_W		900			//	width port 
#define PORT_BMP_H		900			//	height port

//------------------------------------------------------------------------------
// GLOBAL STRUCTURE
//------------------------------------------------------------------------------
typedef int bool;

typedef struct ship 				
{
	float x, y;					//	location coordinates
	float traj_grade; 			//	inclination angle
	float vel;					//	speed
	struct timespec p_time;		//	parking time
	bool parking;				//	if ship is parked
	bool active;				//	if ship is active 
}ship;

typedef struct route 			// struct related to ship identifying its route
{
	BITMAP * trace;				//	trace that the ship must follow
	bool flip;					//	if the trace must be flipped
	int index;					//	position along the trace
	int last_index;				//	last position
}route;

typedef struct place 			//	structure that identify a parking spot
{
	BITMAP * enter_trace;		
	BITMAP * exit_trace;
	int ship_id;				//	ship assigned to a specific place
	bool available;				//	true when a place is not assigned to a ship
}place;

//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
extern BITMAP * sea;						//	bitmap in which are drawn ships			
extern BITMAP * enter_trace[3];				//	array of the ingress trace

//	one place will be assigned to one ship for a certain period of time
extern struct place places[PLACE_NUMBER];
extern struct ship fleet[MAX_SHIPS];		//	fleet of ship
extern struct route routes[MAX_SHIPS];		//	routes[i] related with fleet[i]

// 	positions required by ships. request_access[i] related with fleet[i]
extern int request_access[MAX_SHIPS];
extern int ships_activated;					//	number of ship activated

//	ship allowed to move or not. reply_access[i] related with fleet[i]
extern bool reply_access[MAX_SHIPS];
extern bool end;							// 	true when user presses ESC
extern bool show_routes;					// 	true when user presses SPACE BAR 

// mutex for access to the global variables-------------------------------------
extern pthread_mutex_t mutex_fleet;			//	for fleet
extern pthread_mutex_t mutex_route;			//	for routes
extern pthread_mutex_t mutex_rr;			//	for request_acces & reply_access
extern pthread_mutex_t mutex_p;				//	for places
extern pthread_mutex_t mutex_sea;			//	for sea 
extern pthread_mutex_t mutex_end;			//	for end
extern pthread_mutex_t mutex_s_route;		//	for show_route
extern pthread_mutex_t mutex_s_activated;	//	for ship_activated

//------------------------------------------------------------------------------
//	COMMON FUNCTIONS
//------------------------------------------------------------------------------

//	calulates a random integer in the specified interval
int random_in_range(int min_x, int max_x);

//	checks if the two positions differ for a tolerant space epsilon
bool check_position(float y_ship, int y);

//	safely updates the global variable ships_activated with the given new value
void update_s_activated(int new);

//	get safe way the value of the global variable ships_activated
int get_s_activated();

//	get reply_access value associated to the given ship_id safely
bool get_repl(int ship_id);

//	get request_access value associated to the given ship_id safely
int get_req(int ship_id);

#endif

#ifndef HARBOR_H
#define HARBOR_H
#include <allegro.h>

#define XWIN			1400		// width monitor
#define YWIN			900			// height monitor
#define PERIOD			20			// in ms
#define DLINE			15			// in ms
#define PRIO			10			// priority level
#define AUX_THREAD 		4
#define MAX_THREADS		14			
#define MAX_SHIPS		MAX_THREADS - AUX_THREAD			// max number of ship MUST BE LOWER THAN 30
#define FPS				200.0		
#define FRAME_PERIOD	(1 / FPS)
#define EPSILON			3.f			// guardian distance to the goal
#define ENTER_NUMBER	3
#define	PLACE_NUMBER	8			// number of parking
#define	SEA_COLOR 		makecol(0,85,165)
//------------------------------------------------------------------------------
//	FOR BOOLEAN DEFINITION
//------------------------------------------------------------------------------
#define true			1
#define false			0

//------------------------------------------------------------------------------
//	SHIP GLOBAL CONSTANT
//------------------------------------------------------------------------------
#define XSHIP			18			// width dimension of the ship  
#define YSHIP			54			// height dimension of the ship
#define MIN_P_TIME		50000//1000			// min ship parking time, in ms
#define	MAX_P_TIME		50000//30000		// max ship parking time, in ms
#define MIN_VEL			9//0.5			// minimum speed
#define	MAX_VEL			9			// maximum speed

//-----------------------------------------------------------------------------
// GLOBAL CONSTANTS related to the radar
//------------------------------------------------------------------------------
#define R_BMP_W			451			//	width of the radar
#define R_BMP_H			451			//	height of the radar

#define RMAX			450			//	max radius 
#define ARES			360			// 	max alpha
#define RSTEP			1			//	step of the radius
#define RMIN			0			//	min radius
#define XRAD			450			//x center of the radar = center of the port
#define YRAD			450			//y center of the radar = center of the port

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
	float x, y;
	float traj_grade; 
	float vel;
	struct timespec p_time;
	bool parking;
	bool active;
}ship;

typedef struct triple
{
	float x, y;
	int color;
}triple;

typedef struct array_trace
{
	triple trace[PORT_BMP_W * PORT_BMP_H];
}array_trace;

typedef struct route
{
	BITMAP * trace;
	bool odd;
}route;

typedef struct place 
{
	BITMAP * enter_trace;
	BITMAP * exit_trace;
	int ship_id;
	bool available;
}place; 

//-----------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
extern BITMAP * sea;
extern BITMAP * enter_trace[3];

extern struct place places[PLACE_NUMBER];
extern struct ship fleet[MAX_SHIPS];
extern struct route routes[MAX_SHIPS];
extern struct array_trace m_routes[ENTER_NUMBER + (2 * PLACE_NUMBER)];

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

#endif

#ifndef HARBOR_H
#define HARBOR_H
#include <allegro.h>

#define XWIN			1400		// width monitor
#define YWIN			900			// height monitor
#define PERIOD			20			// in ms
#define DLINE			15			// in ms
#define PRIO			10			// priority level
#define AUX_THREAD 		4
#define MAX_THREADS		15			
#define MAX_SHIPS		MAX_THREADS - AUX_THREAD			// max number of ship MUST BE LOWER THAN 30
#define FPS				200.0		
#define FRAME_PERIOD	(1 / FPS)
#define EPSILON			3.f			// guardian distance to the goal
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
#define MIN_P_TIME		100			// min ship parking time, in ms
#define	MAX_P_TIME		30000		// max ship parking time, in ms

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
#define Y_EXIT			330			//	y value of the port exit
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
	float x_b, y_b;
}route;

typedef struct place 
{
	BITMAP * enter_trace;
	BITMAP * exit_trace;
	int ship_id;
	bool available;
}place; 

extern BITMAP * sea;

extern struct ship fleet[MAX_SHIPS];
extern struct route routes[MAX_SHIPS];

extern int request_access[MAX_SHIPS];

extern bool reply_access[MAX_SHIPS];
extern bool end;
extern pthread_mutex_t mutex_fleet;
extern pthread_mutex_t mutex_route;
extern pthread_mutex_t mutex_rr;
extern pthread_mutex_t mutex_sea;


void * ship_task(void * arg);
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
int make_array_trace(BITMAP * t, pair trace[], int id, bool odd, int req);
bool check_forward(float x_cur, float y_cur, float g_cur);
bool check_position(float y_ship, int y);
int follow_track_frw(int id, int i, pair mytrace[], int last_index, bool move, float vel);
void rotate90_ship(int id, float x_cur, int y1, int y2);
bool exit_ship(int id, float x_cur);
float distance_vector (float x1, float y1, float x2, float y2);
int find_index(pair mytrace[X_PORT * Y_PORT], int posix);
//------------------------------------------------------------------------------
//	USER FUNCTIONS
//------------------------------------------------------------------------------
int click_place(int offset, int delta, int l_x, int r_x);

//------------------------------------------------------------------------------
//	CONTROLLER FUNCTIONS
//------------------------------------------------------------------------------
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
//void terminate();
int random_in_range(int min_x, int max_x);
pair make_pair(int x, int y);

#endif

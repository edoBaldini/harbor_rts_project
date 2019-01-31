#include <allegro.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include "ptask.h"

#define XWIN            1400        // width monitor
#define YWIN            900         // height monitor
#define XSHIP           18          // width dimension of the ship  
#define YSHIP           54          // height dimension of the ship
#define PERIOD          20          // in ms
#define DLINE           25
#define PRIO            10
#define MAX_SHIPS       12          // max number of ship MUST BE LOWER THAN 30
#define MAX_THREADS     32
#define FPS             200.0
#define FRAME_PERIOD    (1 / FPS)
#define EPSILON         3.f        // guardian distance to the goal

#define true            1
#define false           0
//-----------------------------------------------------------------------------
// GLOBAL CONSTANTS related to the radar
//------------------------------------------------------------------------------
#define R_BMP_W         451
#define R_BMP_H         451

#define RMAX            450
#define ARES            360
#define RSTEP           1
#define RMIN            0
#define XRAD            450         //x center of the radar = center of the port
#define YRAD            450         //y center of the radar = center of the port

#define YGUARD_POS      610
#define XPORT           450         //x position of the door port
#define YPORT           505         //y postizion of the doow port

#define PORT_BMP_W      900
#define PORT_BMP_H      900
#define	Y_PLACE 		253
#define Y_EXIT			330

//-----------------------------------------------------------------------------
// GLOBAL STRUCTURE
//------------------------------------------------------------------------------
typedef int bool;

typedef struct ship 
{
	float x, y;
	float traj_grade; 
	BITMAP * boat;
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

struct place places[8];
struct ship fleet[MAX_SHIPS];
struct route routes[MAX_SHIPS];

//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
BITMAP * sea;
BITMAP * radar;
int sea_color;
int ships_activated = 0;
bool end = false;
int aux_thread = 0;

int request_access[MAX_SHIPS];
bool reply_access[MAX_SHIPS];


//------------------------------------------------------------------------------
// FUNCTIONS FOR RADAR
//------------------------------------------------------------------------------

bool check_ship(int j, int color)
{
	int  actual_color = 255 - (j * 10);
	return color == makecol(0,0, actual_color);
}

void * radar_task(void * arg)
{   

// Task private variables
bool found;
float a = 0.f;
float alpha;
int d = 0;
int x, y, j;
int color;
int r_col = makecol(255, 255, 255);
const int id = get_task_index(arg);
	set_activation(id);

	while (!end) 
	{   
		alpha = a * M_PI / 180.f;   // from degree to radiants
		for (d = RMIN; d < RMAX; d += RSTEP)
		{
			x = XRAD + d * cos(alpha);
			y = YRAD - d * sin(alpha);
			color = getpixel(sea, x, y);
			for (j = 0; j <= ships_activated; j++)
			{
				found = check_ship(j, color);

				if (found)
				{
					putpixel(radar, (x / 2), (y / 2), r_col);
			   }
			}
		}
//from the formula L = pi * r *a / 180 I can guarantee that, the circumference
//arc len is less than the width label in this way the ships will be always seen
		a += 1; 
		if (a == 360.0)
		{
			a = 0.0;
			clear_bitmap(radar);
			circle(radar, R_BMP_W / 2, R_BMP_H / 2, R_BMP_H / 2, r_col);
		}
	
		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! radar\n", id);
		}
		wait_for_activation(id);
	}

return NULL;
}


pair make_pair(int x, int y)
{
	pair coordinates;
	coordinates.x = x;
	coordinates.y = y;
	return coordinates;
}

float degree_fix(float grade)
{
	int new_grade = (grade > 0 ? grade : (2 * M_PI + grade)) * 360 / (2 * M_PI); //from radiants to degree 360
	return (new_grade * 256 / 360);
}

void fill_places(BITMAP * t_bmp1, BITMAP * t_bmp2)
{
int i;
BITMAP * t[13];

t[0] = create_bitmap(PORT_BMP_W, PORT_BMP_H);
clear_to_color(t[0], makecol(255,0,255));
t[0] = t_bmp1;
places[0].enter_trace = t[0];
places[0].exit_trace = create_bitmap(PORT_BMP_W, PORT_BMP_H);
places[0].exit_trace = load_bitmap("e1.bmp", NULL);
places[0].available = true;

//t[1] = create_bitmap(PORT_BMP_W, PORT_BMP_H);
//clear_bitmap(t[1]);
//places[1].enter_trace = t[1];
places[1].enter_trace = create_bitmap(PORT_BMP_W, PORT_BMP_H);
clear_to_color(places[1].enter_trace, makecol(255,0,255));
draw_sprite_h_flip(places[1].enter_trace, t[0],0,0);
places[1].available = true;

/*t[1] = create_bitmap(PORT_BMP_W, PORT_BMP_H);
clear_bitmap(t[1]);
t[1] = t_bmp2;
places[2].enter_trace = t[1];
places[3].enter_trace = t[1];
draw_sprite_h_flip(places[3].enter_trace, t[1], 0,0);*/
	/*for (i = 0; i < 13; i++)
	{
		t[i] = create_bitmap(PORT_BMP_W, PORT_BMP_H);
		clear_to_color(t[i], makecol(255,0,255));
		

		if (i == 0)
		{
			t[i] = t_bmp;
			places[0].enter_trace = t[i];
		}

		if (i % 2 != 0)
		{
			places[i].enter_trace = t[i];
			draw_sprite_h_flip(places[i].enter_trace, t[i - 1] , 0, 0);
		}
		else if(i > 0)
		{
			pivot_scaled_sprite(t[i], t[i - 2], XPORT, YPORT, XPORT, YPORT, ftofix(degree_fix(-0.06)),61000);
			places[i].enter_trace = t[i];
		}

		places[i].available = true;

	}*/

}

void reverse_array(pair trace[XPORT * YPORT], int last_index)
{
int i;
pair aux;
int size = last_index;
	for(i = 0; i < size / 2; ++i)
	{
		aux = trace[i];
		trace[i] = trace[last_index];
		trace[last_index] = aux;
		last_index --;
	}
}


void make_array_trace(BITMAP * t, pair trace[XPORT * YPORT], int id, bool odd)
{
int color;
int index = 0;
int i, j;
int last_index;
for (j = PORT_BMP_H; j > 0; --j)   
	{
		if (odd)
		{
			for (i = PORT_BMP_W; i > 0 ; --i)
			{
				color = getpixel(t, i, j);
				if (color == 0)
				{
					trace[index] = make_pair(i,j);
					index ++;
				}
			}
		}
		else
		{	
			for (i = 0; i < PORT_BMP_W; ++i)
			{
				int color = getpixel(t, i, j);
				if (color == 0)
				{
					trace[index] = make_pair(i,j);
					index ++;
				}
			}
		}
	}
	last_index = -- index;

	if (request_access[id] == Y_EXIT)
		reverse_array(trace, last_index);

	routes[id].x = trace[last_index].x;
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR SHIPS
//------------------------------------------------------------------------------
int random_in_range(int min_x, int max_x)
{
	return rand() % (max_x + 1 - min_x) + min_x;
}

float degree_rect(float x1, float y1, float x2, float y2)
{   
	float angular_coefficient = (x1 == x2) ? x1 : ((y2 - y1) / (x2 - x1));
	float degree = atanf(angular_coefficient);
	return (x2 <= x1) ? degree + M_PI  : degree + 2 * M_PI;

}

void follow_track_frw(int id, int i, pair mytrace[XPORT * YPORT])
{

	fleet[id].traj_grade = degree_rect(fleet[id].x, fleet[id].y, 
						mytrace[i + 60].x, mytrace[i + 60].y);
	fleet[id].x = mytrace[i].x;
	fleet[id].y = mytrace[i].y;
	
}

void rotate90_ship(int id, int y1, int y2)
{
float aux = 5000/PERIOD;
	fleet[id].y += (y2 - y1) / aux;
	fleet[id].traj_grade -= (M_PI / 2) / aux;
}

 bool check_forward(int id)
 {
 int i, j;
 int color;


 		color = getpixel(sea, fleet[id].x, fleet[id].y - 60);

 		if (color != sea_color && color != -1){
 			return true;
 		}
 	
 	return false;
 }

 bool check_spec_position(int id, float x, float y)
{
	return fabs(fleet[id].x - x) <= EPSILON &&
			fabs(fleet[id].y - y) <= EPSILON;
}

bool check_yposition(int id, int y)
{
	return fabs(fleet[id].y - y) <= EPSILON;
}

 void * ship_task(void * arg)
 {

int i;
int ship_id;
pair mytrace[XPORT * YPORT];
struct timespec dt;
struct timespec now;
bool mytrace_computed;
bool first_step, second_step, third_step, fourth_step;
bool move = true;
bool wait = false;
	// Task private variables
const int id = get_task_index(arg);
	set_activation(id);
	ship_id 			= id - aux_thread;
	first_step			= true;
	second_step 		= false;
	third_step 			= false;
	fourth_step			= false;
	mytrace_computed 	= false;
	i 					= 0;

	while (!end) 
	{
		move = (check_forward(ship_id)) ? false: true;

		if (!mytrace_computed)
		{  
			i = 0;
			make_array_trace(routes[ship_id].trace, mytrace, ship_id, 
								routes[ship_id].odd);
			mytrace_computed = true;
		}
		if (first_step)
		{	

			if(check_yposition(ship_id, YGUARD_POS))
			{
				request_access[ship_id] = YPORT;
				second_step = true;
				first_step = false;
			}

			else
			{
				follow_track_frw(ship_id, i, mytrace);
				i++;
			}

		}

		if (second_step && reply_access[ship_id])
		{
			if(check_yposition(ship_id, YPORT))
			{
				request_access[ship_id] = Y_PLACE;
				i = 0;
				mytrace_computed = false;
				third_step = true;
				second_step = false;
				reply_access[ship_id] = false;
			}

			else
			{
				follow_track_frw(ship_id, i, mytrace);
				i++;
			}

		}

		if (third_step && reply_access[ship_id])
		{
			if(check_yposition(ship_id, Y_PLACE - YSHIP))
			{
				if (!wait)
				{
					clock_gettime(CLOCK_MONOTONIC, &dt);
					time_add_ms(&dt, 2000);
					wait = true;
				}
				else
				{
					clock_gettime(CLOCK_MONOTONIC, &now);
					if (time_cmp(now, dt) >= 0)
					{
						request_access[ship_id] = Y_EXIT;
						reply_access[ship_id] = false;
						mytrace_computed = false;
						third_step = false;
						fourth_step = true;
					}
				}
			}

			else
			{
				follow_track_frw(ship_id, i, mytrace);
				i++;
			}

		}

		if (fourth_step && reply_access[ship_id])
		{
			if (fleet[ship_id].traj_grade > M_PI && fleet[ship_id].y < Y_EXIT)
				rotate90_ship(ship_id, Y_PLACE, Y_EXIT + YSHIP);

			else
			{
				follow_track_frw(ship_id, i, mytrace);
				i++;
			}
		}

		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! ship\n", id);
		}
		wait_for_activation(id);

	}

	return NULL;
 }

bool try_access_port()
{
int j;
	for (j = 0; j < ships_activated; ++j)
		{
			if (check_yposition(j, YGUARD_POS))
			{
				routes[j].y = YPORT;
				routes[j].x = XPORT;
				reply_access[j] = true;
				return j;
			}
		}
	return -1;
 }

bool assign_trace(int ship)
 {
 int i;
	for (i = 0; i < 8; i++)
	{         
		if (places[i].available)
		{	
			routes[ship].trace = places[i].enter_trace;
			routes[ship].y = Y_PLACE;
			routes[ship].odd = ( i % 2 != 0);
			places[i].available = false;
			places[i].ship_id = ship;
			return true;
		}

		else if (places[i].ship_id == ship)
		{	
			routes[ship].trace = places[i].exit_trace;
			routes[ship].y = YPORT;
			//places[i].available = true;
			//places[i].ship_id = -1;
			return true;
		}
	} 
	return false;
 }

void * controller_task(void *arg)
{
int ship;
int i = 0;
bool access_port = true;
bool access_place = true;


const int id = get_task_index(arg);
	set_activation(id);

	while (!end) 
	{
		if (request_access[i] == YPORT)
		{
			if (access_port)
			{
				routes[i].x = XPORT;
				routes[i].y = YPORT;
				reply_access[i] = true;
				access_port = false;
			}
		}

		if (request_access[i] == Y_PLACE)
		{
			if (!access_port && access_place && assign_trace(i))
			{
				reply_access[i] = true;
				access_place = false;
				access_port = true;
			}
			//else reply_access[i] = false;
		}

		
		if (check_yposition(i, Y_PLACE))
			access_place = true;

		if (request_access[i] == Y_EXIT && access_place)
		{ 
			if (assign_trace(i))
			{
				reply_access[i] = true;
				access_place = false;
			}
			//else reply_access[i] = false;
		}

		i = (i < ships_activated) ? i + 1 : 0;
		
		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! ship\n", id);
		}
		wait_for_activation(id);

	}

	return NULL;
}
	
void * display(void *arg)
{
BITMAP * port_bmp;
BITMAP * back_sea_bmp;
BITMAP * trace1;
BITMAP * trace2;
int i;

	back_sea_bmp = create_bitmap(PORT_BMP_W, PORT_BMP_H);
	sea = create_bitmap(PORT_BMP_W, PORT_BMP_H);
	radar = create_bitmap(R_BMP_W, PORT_BMP_H);

	sea_color = makecol(0,85,165);

	clear_bitmap(back_sea_bmp);
	clear_to_color(sea, sea_color);
	clear_bitmap(radar);

	port_bmp = load_bitmap("port.bmp", NULL);
	trace1 = load_bitmap("w1.bmp", NULL);
	trace2 = load_bitmap("w1.bmp", NULL);
	circle(radar, R_BMP_W / 2, R_BMP_H / 2, R_BMP_H / 2, makecol(255, 255, 255));
	fill_places(trace1, trace2);
	// Task private variables
	const int id = get_task_index(arg);
	set_activation(id);

	while (!end) {

		clear_to_color(sea, sea_color);
		for (i = 0; i < ships_activated; i++ )
			pivot_sprite(sea, fleet[i].boat, fleet[i].x, fleet[i].y, 
							fleet[i].boat-> w / 2, 0, itofix(degree_fix(fleet[i].traj_grade)+64));

		blit(sea, back_sea_bmp, 0, 0, 0,0,sea->w, sea->h);
		draw_sprite(back_sea_bmp, port_bmp, 0, 0);
		for (int k = 0; k < 1; k++)
			if (routes[0].trace != NULL)
				draw_sprite(back_sea_bmp, routes[0].trace, 0,0);

		putpixel(back_sea_bmp, XPORT, YPORT, makecol(255,0,255));
		blit(back_sea_bmp, screen, 0,0,0,0,back_sea_bmp->w, back_sea_bmp->h); 
		blit(radar, screen, 0, 0,910, 0, radar->w, radar->h);

		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! display\n", id);
		}
		wait_for_activation(id);

	}

	for (i = 0; i < ships_activated; i++)
		destroy_bitmap(fleet[i].boat);
	
	destroy_bitmap(port_bmp);
	destroy_bitmap(sea);
	destroy_bitmap(radar);

	return NULL;
 }

void init(void)
 {

	allegro_init();
	install_keyboard();
	set_color_depth(16);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN,0,0);
	
	task_create(display, PERIOD, DLINE, PRIO);
	aux_thread ++;
	task_create(radar_task, 3, 6, PRIO);
	aux_thread ++;
	task_create(controller_task, PERIOD, DLINE, PRIO);
	aux_thread ++;

 }

void mark_label(BITMAP * boat)
{

	int color_assigned = 255 - (ships_activated * 10);
	rectfill(boat, 5,7, 12, 14, makecol(0,0, color_assigned));
	printf("color assigned %d\n", makecol(0,0, color_assigned));

}

void init_ship()
 {

int actual_index = ships_activated + 1; 
	if (actual_index <= MAX_SHIPS)
	{
		printf("hi new ship n* %d\n", ships_activated);
		fleet[ships_activated].boat = create_bitmap(XSHIP, YSHIP);
		fleet[ships_activated].boat = load_bitmap("ship_c.bmp", NULL);
		mark_label(fleet[ships_activated].boat);
		fleet[ships_activated].x = ((ships_activated * 144) % 864) + 54; //(ships_activated * 55 + 150) % PORT_BMP_W;
		fleet[ships_activated].y = random_in_range(PORT_BMP_H, YWIN);
		routes[ships_activated].x = XPORT;
		routes[ships_activated].y = YGUARD_POS;
		routes[ships_activated].trace = load_bitmap("t1_a.bmp", NULL);
		printf("%f\n", fleet[ships_activated].x);
		ships_activated += 1;

		task_create(ship_task, PERIOD, DLINE, PRIO);
	}
 }

int main()
{

char scan;
int i;

	if (MAX_SHIPS + aux_thread > MAX_THREADS)
	{
		printf("too many ships! The max number of ships + the auxiliar thread"
		" must be lower than max number of thread (32)\n");
		return 0;
	}

	init();

	do {
		scan = 0;
		if (keypressed()) scan = readkey() >> 8;
		if (scan == KEY_ENTER){
			printf("try to create new ship\n");
			init_ship();
			i++;
		}
	} while (scan != KEY_ESC);

	end = true;
	wait_tasks();

	return 0;
}
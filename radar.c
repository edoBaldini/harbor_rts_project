#include <allegro.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include "ptask.h"

#define XWIN            1400        // width monitor
#define YWIN            680         // height monitor
#define XSHIP           18          // width dimension of the ship  
#define YSHIP           54          // height dimension of the ship
#define PERIOD          40          // in ms
#define DLINE           45
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
#define VEL             150

#define PORT_BMP_W      900
#define PORT_BMP_H      900
#define	Y_PLACE 		253
//-----------------------------------------------------------------------------
// GLOBAL STRUCTURE
//------------------------------------------------------------------------------
typedef int bool;

typedef struct ship 
{
	float x, y;
	float vel;
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
	float x, y; 
}route;

typedef struct place 
{
	BITMAP * trace;
	bool available;
}place; 

struct place places[13];
struct ship fleet[MAX_SHIPS];
struct route routes[MAX_SHIPS];
//struct pair trace[PORT_BMP_W * PORT_BMP_H];
//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
BITMAP * sea;
BITMAP * radar;
int sea_color;
int ships_activated = 0;
bool end = false;
bool sem = true;
int aux_thread = 0;

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

void print_matrix(int matrix[ARES][RMAX])
{
int i = 0;
int j = 0;
int r, g, b, c;

	for(; i < ARES; ++i)
	{
		for(j = 0; j < RMAX; ++j)
		{
			c = matrix[i][j];
			r = getr(c);
			g = getg(c);
			b = getb(c);
			printf("(%d,%d,%d)\n", r, g, b);
		}
	}
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

void fill_places(BITMAP * t_bmp)
{
int i;
BITMAP * t[13];
	for (i = 0; i < 13; i++)
	{
		t[i] = create_bitmap(PORT_BMP_W, PORT_BMP_H);
		clear_to_color(t[i], makecol(255,0,255));
		

		if (i == 0)
		{
			t[i] = t_bmp;
			places[0].trace = t[i];
		}

		if (i % 2 != 0)
		{
			places[i].trace = t[i];
			draw_sprite_h_flip(places[i].trace, t[i - 1] , 0, 0);
		}
		else if(i > 0)
		{
			pivot_scaled_sprite(t[i], t[i - 2], XPORT, YPORT, XPORT, YPORT, ftofix(degree_fix(-0.06)),61000);
			places[i].trace = t[i];
		}

		places[i].available = true;

	}

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

void make_array_trace(BITMAP * t, pair trace[XPORT * YPORT], int id)
{
float y_place;
int index = 0;
int place_index = 0;
int i, j;
int last_index;

	for (i = 0; i < PORT_BMP_W; ++i)

	{
		for (j = PORT_BMP_H; j > 0; --j)    

		{
			int color = getpixel(t, i, j);

			if (color == 0)
			{
				trace[index] = make_pair(i,j);
				index ++;
			}

		}
	}
	last_index = -- index;
	
	if(trace[0].y < trace[last_index].y)
		reverse_array(trace, last_index);

	routes[id].x = trace[last_index].x;
	//routes[id].y = 253.0;//trace[last_index].y;
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR SHIPS
//------------------------------------------------------------------------------
int random_in_range(int min_x, int max_x)
{
	return rand() % (max_x + 1 - min_x) + min_x;
}

float distance_vector(float x_start, float y_start, 
						float x_target, float y_target)
{
	float x_m = x_target - x_start;
	float y_m = y_target - y_start;
	
	return sqrtf((x_m * x_m) + (y_m * y_m));
}

float degree_rect(float x1, float y1, float x2, float y2)
{   
	float angular_coefficient = (x1 == x2) ? x1 : ((y2 - y1) / (x2 - x1));
	float degree = atanf(angular_coefficient);
	return (x2 <= x1) ? degree + M_PI  : degree + 2 * M_PI;

}

float actual_vel(float x, float y, 
					float xtarget_pos, float ytarget_pos, float s_vel,bool acc)
{
float velx;
float vely;
float vel;

	velx = 2 * (xtarget_pos - x);
	vely = 2 * (ytarget_pos - y);
	vel = (sqrtf(velx * velx + vely * vely));
	if (acc)
		return (fabs(vel) < s_vel) ? s_vel : vel;
	else return (fabs(vel) > s_vel) ? s_vel : vel; 
}

void desired_degree(int id, float desired_degree)
{

}

float xlinear_movement(float x, float xtarget_pos, float vel, float degree)
{   
	if (fabs(x - xtarget_pos) <= EPSILON)
		return x;
	else
		return (x + (vel * cos(degree) / PERIOD));
 }

float ylinear_movement(float y, float ytarget_pos, float vel, float degree)
{   
	if (fabs(y - ytarget_pos) <= EPSILON)
		return y;
	else
		return (y + (vel * sin(degree) / PERIOD));
 }

 bool check_forward(int id)
 {
 int i, j;
 int color;
 bool found;

 	for (j = 0; j < 60; ++j)
 	{
 		color = getpixel(sea, fleet[id].x, fleet[id].y - j);
 		for (i = 0; i < ships_activated; ++i)
 		{
 			found = check_ship(i, color);
 			if (found)
 				return true;
 		}
 	}
 	return false;
 }

 void * ship_task(void * arg)
 {

bool need_stop = true; // MUST BE CHANGED!!!!
bool found_ship = false;
bool mytrace_computed = false;
bool for_place = false;
int i, j = 0;
int ship_id;
int color;
pair mytrace[XPORT * YPORT];
int index = 0;
float acc;
float target_degree;
float objective;
ship * myship;
route * myroute;

	// Task private variables
	const int id = get_task_index(arg);
	set_activation(id);
	ship_id = id - aux_thread;
	myship = &fleet[ship_id];
	myroute = &routes[ship_id]; 
	myship-> vel = 60;
	//myship->x = trace[0].x;
	//myship->y = trace[0].y;   
	while (!end) 
	{
		found_ship = check_forward(ship_id);

		if (myroute-> trace == NULL)
		{
			if (!found_ship){
		
			myship-> vel = 100;
			myship-> traj_grade = degree_rect(myship-> x, myship-> y, 
													myroute-> x, myroute-> y);
			
			myship-> x = xlinear_movement(myship-> x, myroute-> x, myship-> vel, 
												myship-> traj_grade);
			
			myship-> y = ylinear_movement(myship-> y, myroute-> y, myship-> vel, 
												myship-> traj_grade);
		}


		}

		else 
		{
			if (!mytrace_computed)
			{
				make_array_trace(myroute-> trace, mytrace, ship_id);
				mytrace_computed = true;
				for_place = (myroute-> y == Y_PLACE) ? true : false; 
			}

			objective =  (myship-> vel) * FRAME_PERIOD; //sqrtf((myship-> x * myship-> x) + (myship->y * myship-> y)) + myship-> vel * PERIOD * FRAME_PERIOD;
		
			acc = distance_vector(myship-> x, myship-> y, mytrace[i].x, mytrace[i].y);
			if (acc >= objective)
			{
				myship-> traj_grade = degree_rect(myship-> x, myship-> y, 
										mytrace[i + 60].x, mytrace[i + 60].y);//degree_rect(myship-> x, myship-> y, trace[i].x, trace[i].y);//;
				
				if(mytrace[i + 60].x == 0)
					myship-> traj_grade = degree_rect(myship-> x, myship-> y, 
												mytrace[i].x, mytrace[i].y);
			
				myship->x = mytrace[i].x;
				myship->y = mytrace[i].y;
				acc = 0;

			}
			i++;

			if (myship-> y <= myroute-> y){
				myship->traj_grade = - M_PI / 2;
				myship-> y = ylinear_movement(myship->y, myroute-> y - YSHIP, myship-> vel, myship-> traj_grade);
			}

			if (for_place)
			{
				if (fabs(myship-> x - myroute-> x) <= EPSILON && fabs(myship-> y - (myroute-> y - YSHIP)) <= EPSILON)
				{
					sleep(5);
				}
			}
			else if (fabs(myship-> x - myroute-> x) <= EPSILON && fabs(myship-> y - myroute-> y) <= EPSILON)
			{
				mytrace_computed = false;
				i = 0;
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

bool check_spec_position(int id, float x, float y)
{
	return fabs(fleet[id].x - x) <= EPSILON &&
			fabs(fleet[id].y - y) <= EPSILON;
}

bool check_position(int id)
{
	return fabs(fleet[id].x - routes[id].x) == EPSILON &&
			fabs(fleet[id].y - routes[id].y) == EPSILON;
}

bool try_access_port()
{
int j;
	for (j = 0; j < ships_activated; ++j)
		{
			if (fabs(YGUARD_POS - fleet[j].y) <= EPSILON)
			{
				routes[j].y = YPORT;
				routes[j].x = XPORT;
				//fleet[j].vel = 100.0;
				routes[j].trace =  load_bitmap("w1.bmp", NULL);

				return j;
			}
		}
	return -1;
 }

bool assign_trace(int first_trace)
 {
 int i;
	for (i = 0; i < 13; i++)
	{           
		if (places[i].available)
		{
			routes[first_trace].trace = places[i].trace;
			routes[first_trace].y = Y_PLACE;
			places[i].available = false;
			return true;
		}
	} 
	return false;
 }

void * controller_task(void *arg)
{
int i, j;
int first_trace, second_trace;
bool access_port = true;
bool access_route = false;
const int id = get_task_index(arg);
	set_activation(id);

	while (!end) {
		if (access_port)
		{
			first_trace = try_access_port();
			if (first_trace >= 0)
				access_port = false;
		}

	   else
		{
			if (check_spec_position(first_trace, XPORT, YPORT))
			{
				//fleet[i].traj_grade = (- M_PI / 2);
				access_route = assign_trace(first_trace);
				if (access_route)
				{
					access_port = true;
					second_trace = first_trace;
					first_trace = -1;     
				}

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
	
void * display(void *arg)
{
BITMAP * port_bmp;
BITMAP * back_sea_bmp;
BITMAP * trace;
int i;

	back_sea_bmp = create_bitmap(PORT_BMP_W, PORT_BMP_H);
	sea = create_bitmap(PORT_BMP_W, PORT_BMP_H);
	radar = create_bitmap(R_BMP_W, PORT_BMP_H);

	sea_color = makecol(0,85,165);

	clear_bitmap(back_sea_bmp);
	clear_to_color(sea, sea_color);
	clear_bitmap(radar);

	port_bmp = load_bitmap("port.bmp", NULL);
	trace = load_bitmap("t2.bmp", NULL);
	circle(radar, R_BMP_W / 2, R_BMP_H / 2, R_BMP_H / 2, makecol(255, 255, 255));
	fill_places(trace);
	// Task private variables
	const int id = get_task_index(arg);
	set_activation(id);

	while (!end) {

		clear_to_color(sea, sea_color);
		for (i = 0; i < ships_activated; i++ )
			pivot_sprite(sea, fleet[i].boat, fleet[i].x, fleet[i].y, 
							fleet[i].boat-> w / 2, 0, ftofix(degree_fix(fleet[i].traj_grade)+64));

		blit(sea, back_sea_bmp, 0, 0, 0,0,sea->w, sea->h);
		draw_sprite(back_sea_bmp, port_bmp, 0, 0);
		//for (int k = 0; k < 13; k++)
			//draw_sprite(back_sea_bmp, places[0].trace, 0,0);
		//draw_sprite(back_sea_bmp, places[1].trace, 0,0);

		//pivot_scaled_sprite(back_sea_bmp, t[0], XPORT, YPORT, XPORT, YPORT, ftofix(degree_fix(-0.06)), 61000);
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
		//fleet[ships_activated].x = random_in_range(0, PORT_BMP_W);
		fleet[ships_activated].x = ((ships_activated * 144) % 864) + 54; //(ships_activated * 55 + 150) % PORT_BMP_W;
		fleet[ships_activated].y = random_in_range(PORT_BMP_H, YWIN);
		//fleet[ships_activated].vel = VEL;
		routes[ships_activated].x = fleet[ships_activated].x;
		routes[ships_activated].y = YGUARD_POS;
		routes[ships_activated].trace = NULL;
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
#include "harbor.h"
#include <math.h>
#include <stdio.h>
#include "ptask.h"

#define XWIN			1400		// width monitor
#define YWIN			700			// height monitor
#define PERIOD			10			// in ms
#define DLINE			15			// in ms
#define PRIO			10			// priority level
#define MAX_SHIPS		12			// max number of ship MUST BE LOWER THAN 30
#define MAX_THREADS		32			
#define FPS				200.0		
#define FRAME_PERIOD	(1 / FPS)
#define EPSILON			3.f			// guardian distance to the goal

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
#define P_TIME			2000		// ship parking time, in ms

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

//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
struct place places[8];
struct ship fleet[MAX_SHIPS];
struct route routes[MAX_SHIPS];

BITMAP * sea;
BITMAP * radar;
int sea_color;
int ships_activated = 0;
bool end = false;
int aux_thread = 0;

int request_access[MAX_SHIPS];
bool reply_access[MAX_SHIPS];


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


void * ship_task(void * arg)
 {

int i;
int ship_id;
int last_index;
pair mytrace[X_PORT * Y_PORT];
struct timespec dt;
struct timespec now;
bool mytrace_computed;
bool first_step, second_step, third_step, fourth_step;
bool move = true;
bool wait = false;
bool termination = false;
	// Task private variables
const int id = get_task_index(arg);
	set_activation(id);
	printf(" id task %d\n", id);
	ship_id 			= id - aux_thread;
	first_step			= true;
	second_step 		= false;
	third_step 			= false;
	fourth_step			= false;
	mytrace_computed 	= false;
	i 					= 0;

	while (!end && !termination) 
	{
		
		move = (check_forward(ship_id)) ? false: true;
		if (first_step)
		{	

			if (!mytrace_computed)
			{
				last_index = make_array_trace(routes[ship_id].trace, mytrace, ship_id, 
								routes[ship_id].odd);
				mytrace_computed = true;
				i = 0;
			}

			if(check_yposition(ship_id, YGUARD_POS))
			{
				request_access[ship_id] = Y_PORT;
				second_step = true;
				first_step = false;
			}

			else if (move)
			{
				follow_track_frw(ship_id, i, mytrace, last_index);
				i++;
			}

		}

		if (second_step && reply_access[ship_id])
		{
			if(check_yposition(ship_id, Y_PORT))
			{
				reply_access[ship_id] = false;
				request_access[ship_id] = Y_PLACE;
				i = 0;
				mytrace_computed = false;
				third_step = true;
				second_step = false;
			}

			else
			{
				follow_track_frw(ship_id, i, mytrace, last_index);
				i++;
			}

		}

		if (third_step && reply_access[ship_id])
		{
			if (!mytrace_computed)
			{
				last_index = make_array_trace(routes[ship_id].trace, mytrace, ship_id, 
								routes[ship_id].odd);
				mytrace_computed = true;
				i = 0;
			}

			if(check_yposition(ship_id, Y_PLACE - YSHIP))
			{

				if (!wait)
				{
					clock_gettime(CLOCK_MONOTONIC, &dt);
					time_add_ms(&dt, P_TIME);
					wait = true;
				}
				else
				{
					clock_gettime(CLOCK_MONOTONIC, &now);
					if (time_cmp(now, dt) >= 0)
					{
						reply_access[ship_id] = false;
						request_access[ship_id] = Y_EXIT;
						mytrace_computed = false;
						third_step = false;
						fourth_step = true;
					}
				}
			}

			else
			{
				follow_track_frw(ship_id, i, mytrace, last_index);
				i++;
			}

		}

		if (fourth_step && reply_access[ship_id])
		{
			if (!mytrace_computed)
			{
				last_index = make_array_trace(routes[ship_id].trace, mytrace, ship_id, 
								routes[ship_id].odd);
				mytrace_computed = true;
				i = 0;
			}
			if (fleet[ship_id].y < mytrace[0].y)
				rotate90_ship(ship_id, Y_PLACE, Y_EXIT + YSHIP);

			else if(move)
			{
				follow_track_frw(ship_id, i, mytrace, last_index);
				i++;
			}

			if (check_yposition(ship_id, Y_PORT) && request_access[ship_id] != -1)
			{
				reply_access[ship_id] = false;
				request_access[ship_id] = -1;
			}

			if (request_access[ship_id] == -1 && reply_access[ship_id])
			{
				termination = exit_ship(ship_id);
				if (termination)
					fleet[ship_id].active = false;
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

void * controller_task(void *arg)
{
int ship;
int ships_exits;
int i = 0;
bool access_port = true;
bool access_place = true;
bool enter_trace[MAX_SHIPS] = {false};
bool exit_trace[MAX_SHIPS] = {false};

const int id = get_task_index(arg);
	set_activation(id);

	while (!end) 
	{

		if (request_access[i] == Y_PORT)
		{
			if (access_port)
			{
				printf("nave %d entra al porto\n", i);
				routes[i].x = X_PORT;
				routes[i].y = Y_PORT;
				reply_access[i] = true;
				access_port = false;

			}
		}

		if (request_access[i] == Y_PLACE)
		{
			if (access_place && !enter_trace[i])
			{	
				enter_trace[i] = assign_trace(i);

				if (enter_trace[i])
				{
					printf("nave %d assegnato POSTO\n", i);
					reply_access[i] = true;
					access_place = false;
					access_port = true;
					ship = i;
				}
			}
		}

		if (check_yposition(ship, Y_EXIT))
		{
			access_place = true;
			ship = -1;
		}

		if (request_access[i] == Y_EXIT && access_place && !exit_trace[i])
		{ 
			exit_trace[i] = assign_exit(i);
			if (exit_trace[i])
			{
				printf("nave %d in uscita\n", i);
				reply_access[i] = true;
				access_place = false;
			}
		}

		if (request_access[i] == -1 && check_yposition(i, Y_PORT) && !reply_access[i])
		{
			printf("nave %d libera il posto\n", i);
			reply_access[i] = true;
			free_trace(i);
			access_place = true;
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
int i;

	back_sea_bmp = create_bitmap(PORT_BMP_W, PORT_BMP_H);
	sea = create_bitmap(PORT_BMP_W, PORT_BMP_H);
	radar = create_bitmap(R_BMP_W, PORT_BMP_H);

	sea_color = makecol(0,85,165);

	clear_bitmap(back_sea_bmp);
	clear_to_color(sea, sea_color);
	clear_bitmap(radar);

	port_bmp = load_bitmap("port.bmp", NULL);
	circle(radar, R_BMP_W / 2, R_BMP_H / 2, R_BMP_H / 2, makecol(255, 255, 255));
	fill_places();
	// Task private variables
	const int id = get_task_index(arg);
	set_activation(id);

	while (!end) {

		clear_to_color(sea, sea_color);
		for (i = 0; i < ships_activated; ++i)
			pivot_sprite(sea, fleet[i].boat, fleet[i].x, fleet[i].y, 
							fleet[i].boat-> w / 2, 0, itofix(degree_fix(fleet[i].traj_grade)+64));

		blit(sea, back_sea_bmp, 0, 0, 0,0,sea->w, sea->h);
		draw_sprite(back_sea_bmp, port_bmp, 0, 0);
		
//	USEFUL TO VISUALIZE THE TRACKS THAT THE SHIPS ARE FOLLOWING
		//for (int k = 0; k < ships_activated; k++)
		//	if (routes[0].trace != NULL)
		//		draw_sprite(back_sea_bmp, routes[k].trace, 0,0);
		
//	USEFUL TO VISUALIZE CHECK_FWD()
		/*for (int k = 0; k < ships_activated; ++k)
			for (int j = 2; j < 70; j++)
				putpixel(back_sea_bmp, fleet[k].x + j * cos(fleet[k].traj_grade), fleet[k].y + j * sin(fleet[k].traj_grade), 0);*/

		putpixel(back_sea_bmp, X_PORT, Y_PORT, makecol(255,0,255));
		blit(back_sea_bmp, screen, 0,0,0,0,back_sea_bmp->w, back_sea_bmp->h); 
		blit(radar, screen, 0, 0,910, 0, radar->w, radar->h);

		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! display\n", id);
		}
		wait_for_activation(id);

	}

	for (i = 0; i < ships_activated; ++i)
		destroy_bitmap(fleet[i].boat);
		

	for (i = 0; i < 8; ++i)
	{
		destroy_bitmap(places[i].enter_trace);
		destroy_bitmap(places[i].exit_trace);
	}
	
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

void init_ship()
{

int index = (ships_activated % 3);
BITMAP * enter_trace[3];

enter_trace[0] = load_bitmap("e1.bmp", NULL);
enter_trace[1] = load_bitmap("e2.bmp", NULL);
enter_trace[2] = load_bitmap("e3.bmp", NULL);

	if (ships_activated < MAX_SHIPS)
	{
		//printf("hi new ship n* %d\n", ships_activated);
		fleet[ships_activated].boat = create_bitmap(XSHIP, YSHIP);
		fleet[ships_activated].boat = load_bitmap("ship_c.bmp", NULL);
		mark_label(fleet[ships_activated].boat);
		fleet[ships_activated].x = 0.0; //((ships_activated * 144) % 864) + 54;
		fleet[ships_activated].y = 0.0; random_in_range(PORT_BMP_H, YWIN);
		fleet[ships_activated].active = true;
		routes[ships_activated].x = X_PORT;
		routes[ships_activated].y = YGUARD_POS;
		routes[ships_activated].trace = enter_trace[index]; 
		routes[ships_activated].odd = (index == 1);
		ships_activated += 1;

		task_create(ship_task, PERIOD, DLINE, PRIO);
	}
}

int main()
{
char scan;
int i;
int j;
int ships_terminated[MAX_SHIPS] = {false};

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
			//printf("try to create new ship\n");
			init_ship();
			i++;
		}
			terminate(ships_terminated);

	} while (scan != KEY_ESC);

	end = true;

	terminate(ships_terminated);
	return 0;
}


//------------------------------------------------------------------------------
// FUNCTIONS FOR RADAR
//------------------------------------------------------------------------------

bool check_ship(int j, int color)
{
	int  actual_color = 255 - (j * 10);
	return color == makecol(0,0, actual_color);
}

float degree_rect(float x1, float y1, float x2, float y2)
{   
	float angular_coefficient = (x1 == x2) ? x1 : ((y2 - y1) / (x2 - x1));
	float degree = atanf(angular_coefficient);
	return (x2 <= x1) ? degree + M_PI  : degree + 2 * M_PI;

}

float degree_fix(float grade)
{
	int new_grade = (grade > 0 ? grade : (2 * M_PI + grade)) * 360 / (2 * M_PI); //from radiants to degree 360
	return (new_grade * 256 / 360);
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR SHIPS
//------------------------------------------------------------------------------
void reverse_array(pair trace[X_PORT * Y_PORT], int last_index)
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

int make_array_trace(BITMAP * t, pair trace[PORT_BMP_W * PORT_BMP_H], int id,																	bool odd)
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
	return last_index;
}

bool check_forward(int id)
{
int x, y, j;
int color;

	for (j = 2; j < 70; ++j )
	{
		x = fleet[id].x + j * cos(fleet[id].traj_grade);
		y = fleet[id].y + j * sin(fleet[id].traj_grade);
		color = getpixel(sea, x, y);

		if (color != sea_color && color != -1)
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

void follow_track_frw(int id, int i, pair mytrace[X_PORT * Y_PORT], int last_index)
{
	if(i <= last_index)
	{
		fleet[id].x = mytrace[i].x;
		fleet[id].y = mytrace[i].y;

		if (last_index > i + 60)
			fleet[id].traj_grade = degree_rect(fleet[id].x, fleet[id].y, 
						mytrace[i + 60].x, mytrace[i + 60].y);
		else 
			fleet[id].traj_grade = fleet[id].traj_grade;
	}
}

void rotate90_ship(int id, int y1, int y2)
{
float aux = 5000 / PERIOD;
	
	fleet[id].y += (y2 - y1) / aux;

	if (fleet[id]. x > X_PORT)
		fleet[id].traj_grade -= (M_PI / 2) / aux;
	else
		fleet[id].traj_grade += (M_PI / 2) / aux;
}

bool exit_ship(int id)
{
float aux = 1000 / PERIOD;
	if(fleet[id].x < X_PORT)
	{
		if (fleet[id].x > -YSHIP)
		{
			fleet[id].x -= YSHIP / aux;
			return false;
		}
		else return true;
	}
	else
	{	if (fleet[id].x < PORT_BMP_W + YSHIP)
		{	
			fleet[id].x += YSHIP/ aux;
			return false;
		}
		else return true;
	}
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR CONTROLLER
//------------------------------------------------------------------------------
bool try_access_port()
{
int j;
	for (j = 0; j < ships_activated; ++j)
		{
			if (check_yposition(j, YGUARD_POS))
			{
				routes[j].y = Y_PORT;
				routes[j].x = X_PORT;
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
			routes[ship].y = Y_PLACE - YSHIP;
			routes[ship].odd = ( i % 2 != 0);
			places[i].available = false;
			places[i].ship_id = ship;
			return true;
		}

	} 
	return false;
}

bool assign_exit(int ship)
{
int i;
	for(i = 0; i < 8; i++)
	{
		if (places[i].ship_id == ship)
		{
			routes[ship].trace = places[i].exit_trace;
			routes[ship].y = Y_PORT;
			return true;
		}
	}
	return false;
}

void free_trace(int ship)
{
int i;
	for (i = 0; i < 8; i++)
	{
		if (places[i].ship_id == ship)
		{
			places[i].available = true;
			places[i].ship_id = -1;
		}
	} 
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR AUXILIAR
//------------------------------------------------------------------------------
void fill_places()
{
int i, j;
for (j = 0; j < 8; ++j)
{
	places[j].ship_id = -1;
	places[j].available = false;
}
places[0].enter_trace = load_bitmap("w1.bmp", NULL);
places[0].exit_trace = load_bitmap("x1.bmp", NULL);
places[0].available = true;

places[2].enter_trace = load_bitmap("w2.bmp", NULL);
places[2].exit_trace = load_bitmap("x2.bmp", NULL);
places[2].available = true;

places[1].enter_trace = create_bitmap(PORT_BMP_W, PORT_BMP_H);
clear_to_color(places[1].enter_trace, makecol(255,0,255));
draw_sprite_h_flip(places[1].enter_trace, places[0].enter_trace,0,0);
places[1].exit_trace = create_bitmap(PORT_BMP_W, PORT_BMP_H);
clear_to_color(places[1].exit_trace, makecol(255,0,255));
draw_sprite_h_flip(places[1].exit_trace, places[0].exit_trace,0,0);
places[1].available = true;

places[3].enter_trace = create_bitmap(PORT_BMP_W, PORT_BMP_H);
clear_to_color(places[3].enter_trace, makecol(255,0,255));
draw_sprite_h_flip(places[3].enter_trace, places[2].enter_trace,0,0);
places[3].exit_trace = create_bitmap(PORT_BMP_W, PORT_BMP_H);
clear_to_color(places[3].exit_trace, makecol(255,0,255));
draw_sprite_h_flip(places[3].exit_trace, places[2].exit_trace,0,0);
places[3].available = true;
}

void mark_label(BITMAP * boat)
{

	int color_assigned = 255 - (ships_activated * 10);
	rectfill(boat, 5,7, 12, 14, makecol(0,0, color_assigned));
	//printf("color assigned %d\n", makecol(0,0, color_assigned));

}

void terminate(int ships_terminated[MAX_SHIPS])
{
int i, j;
	for (i = 0; i < ships_activated; ++i)
	{
		if (!fleet[i].active && !ships_terminated[i])
		{
			j = join_spec_thread(i + aux_thread);
			ships_terminated[i] = true;
			printf("joined i %d status %d\n", i + aux_thread, j);
		}
	}

}

int random_in_range(int min_x, int max_x)
{
	return rand() % (max_x + 1 - min_x) + min_x;
}

pair make_pair(int x, int y)
{
	pair coordinates;
	coordinates.x = x;
	coordinates.y = y;
	return coordinates;
}
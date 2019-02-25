#include "common.h"
#include <math.h>
#include <stdio.h>
#include <pthread.h>
#include "ptask.h"
//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
struct place places[PLACE_NUMBER];
struct ship fleet[MAX_SHIPS];
struct route routes[MAX_SHIPS];

BITMAP * sea;
BITMAP * radar;
BITMAP * enter_trace[3];
int ships_activated = 0;
int request_access[MAX_SHIPS];
bool reply_access[MAX_SHIPS];
bool end = false;
bool show_routes = false;

pthread_mutex_t mutex_rr;
pthread_mutex_t mutex_p;
pthread_mutex_t mutex_fleet;
pthread_mutex_t mutex_route;
pthread_mutex_t mutex_sea;
pthread_mutex_t mutex_radar;
pthread_mutex_t mutex_end;
pthread_mutex_t mutex_s_route;

//------------------------------------------------------------------------------
//	RADAR FUNCTIONS
//------------------------------------------------------------------------------
bool check_ship(int color);
float degree_fix(float grade);
void radar_one_line(float alpha);

//------------------------------------------------------------------------------
//	CONTROLLER FUNCTIONS
//------------------------------------------------------------------------------
void fill_trace(int ship, int i, BITMAP * trace);
bool assign_trace(int ship);
bool assign_exit(int ship);
int access_port(int i, int ship_to_port, int ship_to_place);
int access_place(int i, int ship_to_place);
void free_trace(int ship);

//------------------------------------------------------------------------------
//	DISPLAY FUNCTIONS
//------------------------------------------------------------------------------
void view_ships(BITMAP * boat);
void view_routes();

//------------------------------------------------------------------------------
//	AUXILIAR FUNCTIONS
//------------------------------------------------------------------------------
void init(void);
void fill_places();

void * radar_task(void * arg)
{   
bool c_end = false;
float alpha;
float a = 0.f;
const int id = get_task_index(arg);
	set_activation(id);

	while (!c_end) 
	{   
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		alpha = a * M_PI / 180.f;   // from degree to radiants
		radar_one_line(alpha);
		a += 1.8; 

		if (a >= 360.0)
		{
			a = 0.0;
			pthread_mutex_lock(&mutex_radar);
			clear_bitmap(radar);
			pthread_mutex_unlock(&mutex_radar);
		}
		
		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! radar\n", id);
		}
		wait_for_activation(id);
	}

	destroy_bitmap(radar);

return NULL;
}

void * controller_task(void *arg)
{
int i = 0;
int ship_to_port = -1;
int ship_to_place = -1;
bool c_end = false;
const int id = get_task_index(arg);
	set_activation(id);

	while (!c_end) 
	{
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		ship_to_port = access_port(i, ship_to_port, ship_to_place);

		ship_to_place = access_place(i, ship_to_place);
		
		free_trace(i);
		
		i = (i < MAX_SHIPS) ? i + 1 : 0;

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
BITMAP * boat;
int i;
bool c_end = false;

	port_bmp = load_bitmap("port.bmp", NULL);
	boat = load_bitmap("ship_c.bmp", NULL);

	const int id = get_task_index(arg);
	set_activation(id);

	while (!c_end) {
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		clear_to_color(sea, SEA_COLOR);

		view_ships(boat);

		draw_sprite(screen, port_bmp, 0, 0);
		
		if (show_routes)
		{
			view_routes();
		}

		putpixel(screen, X_PORT, Y_PORT, makecol(255,0,255));
		putpixel(screen, X_PORT, YGUARD_POS, makecol(255,0,255));

		pthread_mutex_lock(&mutex_radar);
		circle(radar, R_BMP_W / 2, R_BMP_H / 2, R_BMP_H / 2, makecol(255,255,255));
		blit(radar, screen, 0, 0,910, 0, radar->w, radar->h);
		pthread_mutex_unlock(&mutex_radar);

		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! display\n", id);
		}
		wait_for_activation(id);

	}

	for (i = 0; i < PLACE_NUMBER; ++i)
	{
		destroy_bitmap(places[i].enter_trace);
		destroy_bitmap(places[i].exit_trace);
	}
	
	destroy_bitmap(port_bmp);
	destroy_bitmap(sea);

	return NULL;
}

void init(void)
{

	allegro_init();
	install_keyboard();
	install_mouse();

	set_color_depth(16);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN,0,0);
	set_keyboard_rate(0, 0);
	enable_hardware_cursor();
	show_mouse(screen);
	
	sea = create_bitmap(PORT_BMP_W, PORT_BMP_H);
	radar = create_bitmap(R_BMP_W, PORT_BMP_H);

	//sea_color = makecol(0,85,165);

	clear_to_color(sea, SEA_COLOR);
	clear_bitmap(radar);

	circle(radar, R_BMP_W / 2, R_BMP_H / 2, R_BMP_H / 2, makecol(255, 255, 255));
	fill_places();

	enter_trace[0] = load_bitmap("e1_c.bmp", NULL);
	enter_trace[1] = load_bitmap("e3_c.bmp", NULL);
	enter_trace[2] =  create_bitmap(PORT_BMP_W, PORT_BMP_H);
	
	clear_to_color(enter_trace[2], makecol(255,0,255));
	draw_sprite_h_flip(enter_trace[2], enter_trace[0],0,0);
	
	pthread_mutex_init(&mutex_rr, NULL);
	pthread_mutex_init(&mutex_p, NULL);
	pthread_mutex_init(&mutex_fleet, NULL);
	pthread_mutex_init(&mutex_route, NULL);
	pthread_mutex_init(&mutex_sea, NULL);
	pthread_mutex_init(&mutex_radar, NULL);
	pthread_mutex_init(&mutex_end, NULL);
	pthread_mutex_init(&mutex_s_route, NULL);

	task_create(display, PERIOD	, DLINE, PRIO);
	task_create(radar_task, PERIOD, DLINE, PRIO);
	task_create(controller_task, PERIOD, DLINE, PRIO);
	task_create(user_task, PERIOD, DLINE, PRIO);

}

int main()
{
	init();

	wait_tasks();
	allegro_exit();
	return 0;
}


//------------------------------------------------------------------------------
// FUNCTIONS FOR RADAR
//------------------------------------------------------------------------------

bool check_ship(int color)
{
	int  blue = makecol(0,0, 255);
	return color == blue;
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

void radar_one_line(float alpha)
{
bool found = false;
float x, y;
int d, color;
int r_col = makecol(255, 255, 255);

	for (d = RMIN; d < RMAX; d += RSTEP)
	{
		x = XRAD + d * cos(alpha);
		y = YRAD - d * sin(alpha);

		pthread_mutex_lock(&mutex_sea);
		color = getpixel(sea, x, y);
		pthread_mutex_unlock(&mutex_sea);
		
		found = check_ship(color);
		if (found)
		{
			pthread_mutex_lock(&mutex_radar);
			putpixel(radar, (x / 2), (y / 2), r_col);
			pthread_mutex_unlock(&mutex_radar);
		}
	}
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR CONTROLLER
//------------------------------------------------------------------------------
void fill_trace(int ship, int i, BITMAP * trace)
{
	pthread_mutex_lock(&mutex_route);
	routes[ship].trace = trace;
	routes[ship].y = Y_PLACE - YSHIP;
	routes[ship].odd = (i < 4);
	pthread_mutex_unlock(&mutex_route);
}

bool assign_trace(int ship)
{
int i = random_in_range(0, PLACE_NUMBER -1);
bool available;
BITMAP * enter_trace;

	pthread_mutex_lock(&mutex_p);
	available = places[i].available;
	pthread_mutex_unlock(&mutex_p);

		if (available)
		{	
			pthread_mutex_lock(&mutex_p);
			places[i].available = false;
			places[i].ship_id = ship;
			enter_trace = places[i].enter_trace;
			pthread_mutex_unlock(&mutex_p);

			fill_trace(ship, i, enter_trace);

			return true;
		}

	return false;
}

bool assign_exit(int ship)
{
int i;
int ship_assigned;
BITMAP * exit_trace;
	for(i = 0; i < PLACE_NUMBER; i++)
	{
		pthread_mutex_lock(&mutex_p);
		ship_assigned = places[i].ship_id;
		exit_trace = places[i].exit_trace;
		pthread_mutex_unlock(&mutex_p);
		if (ship_assigned == ship)
		{
			fill_trace(ship, i, exit_trace);
			return true;
		}
	}
	return false;
}

int access_port(int i, int ship_to_port, int ship_to_place)
{
int cur_req;
bool cur_repl;
	
	pthread_mutex_lock(&mutex_rr);
	cur_repl = reply_access[i];
	cur_req = request_access[i];
	pthread_mutex_unlock(&mutex_rr);

	if (ship_to_port == -1 && cur_req == Y_PORT)
	{
		cur_repl = true;
		ship_to_port = (cur_repl) ? i : ship_to_port;

		if (ship_to_port == i)
			printf("ship %d enters to the port\n", i);
	}

	else if (ship_to_port == i && ship_to_place == i)
	{
		ship_to_port = -1;
	}
	
	pthread_mutex_lock(&mutex_rr);
	reply_access[i] = cur_repl;
	pthread_mutex_unlock(&mutex_rr);
	
	return ship_to_port;
}

int access_place(int i, int ship_to_place)
{
int cur_req;
bool cur_repl;
	
	pthread_mutex_lock(&mutex_rr);
	cur_repl = reply_access[i];
	cur_req = request_access[i];
	pthread_mutex_unlock(&mutex_rr);

	if (ship_to_place == -1)
	{
		if (cur_req == Y_PLACE && !cur_repl)
		{
			cur_repl = assign_trace(i);
			ship_to_place = (cur_repl) ? i : ship_to_place;
			
			if (cur_repl)
			{
				printf("ship %d PLACE assigned\n", i);
			}
		}

		if (cur_req == Y_EXIT)
		{
			assign_exit(i);
			ship_to_place = i;
			cur_repl = true;
			printf("ship %d exiting\n", i);
		}
	}
		
	if (ship_to_place == i && (cur_req == 1 || cur_req == -1))
	{
		ship_to_place = -1;
	}

	pthread_mutex_lock(&mutex_rr);
	reply_access[i] = cur_repl;
	pthread_mutex_unlock(&mutex_rr);

	return ship_to_place;
}

void free_trace(int ship)
{
int i;
int cur_req;
	
	pthread_mutex_lock(&mutex_rr);
	cur_req = request_access[ship];
	pthread_mutex_unlock(&mutex_rr);
	
	if (cur_req == -1)
	{
		for (i = 0; i < PLACE_NUMBER; i++)
		{
			pthread_mutex_lock(&mutex_p);
			if (places[i].ship_id == ship)
			{
				places[i].available = true;
				places[i].ship_id = -1;
			}
			pthread_mutex_unlock(&mutex_p);
		}
		cur_req = -2;
	}

	pthread_mutex_lock(&mutex_rr);
	request_access[ship] = cur_req ;
	pthread_mutex_unlock(&mutex_rr); 
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR DISPLAY
//------------------------------------------------------------------------------
void view_ships(BITMAP * boat)
{
int i;
ship cur_ship;
	for (i = 0; i < ships_activated; ++i)
	{
		pthread_mutex_lock(&mutex_fleet);
		cur_ship = fleet[i];
		pthread_mutex_unlock(&mutex_fleet);

		pthread_mutex_lock(&mutex_sea);
		rotate_sprite(sea, boat, cur_ship.x - (XSHIP / 2 ), cur_ship.y,
								 itofix(degree_fix(cur_ship.traj_grade)+64));
		pthread_mutex_unlock(&mutex_sea);
	}

	pthread_mutex_lock(&mutex_sea);
	blit(sea, screen, 0, 0, 0,0,sea->w, sea->h);
	pthread_mutex_unlock(&mutex_sea);
}

void view_routes()
{
int e1, e2, e3;
int i;
int counter;
ship cur_ship;
	for (i = 0; i < ships_activated; ++i)
	{
		e1 = getpixel(screen, 0, 806 + (YSHIP / 2)) == SEA_COLOR;
		e2 = getpixel(screen, 899, 806 + (YSHIP / 2)) == SEA_COLOR;
		e3 = getpixel(screen, 450, 899) == SEA_COLOR;

		pthread_mutex_lock(&mutex_fleet);
		cur_ship = fleet[i];
		pthread_mutex_unlock(&mutex_fleet);

		pthread_mutex_lock(&mutex_route);
		if (cur_ship.y > Y_PORT)
		{
			if ((e1 || e2 || e3) && routes[i].trace != NULL)
			{
				counter ++;
				draw_sprite(screen, routes[i].trace, 0, YSHIP / 2);	
			}
		}
				
		if (cur_ship.y < Y_PORT && routes[i].trace != NULL &&
						 !check_position(cur_ship.y, Y_PLACE - YSHIP)) 
		{
			counter ++;
			draw_sprite(screen, routes[i].trace, 0, YSHIP / 2);	
		}
		pthread_mutex_unlock(&mutex_route);

	}
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR AUXILIAR
//------------------------------------------------------------------------------
void fill_places()
{
int i, j;
	for (j = 0; j < PLACE_NUMBER; ++j)
	{
		places[j].ship_id = -1;
		places[j].available = true;
	}
	places[7].enter_trace = load_bitmap("w1_c.bmp", NULL);
	places[7].exit_trace = load_bitmap("x1_c.bmp", NULL);

	places[6].enter_trace = load_bitmap("w2_c.bmp", NULL);
	places[6].exit_trace = load_bitmap("x2_c.bmp", NULL);

	places[5].enter_trace = load_bitmap("w3_c.bmp", NULL);
	places[5].exit_trace = load_bitmap("x3_c.bmp", NULL);

	places[4].enter_trace = load_bitmap("w4_c.bmp", NULL);
	places[4].exit_trace = load_bitmap("x4_c.bmp", NULL);

	for (i = 0; i < 4; ++i)
	{
		places[i].enter_trace = create_bitmap(PORT_BMP_W, PORT_BMP_H);
		places[i].exit_trace = create_bitmap(PORT_BMP_W, PORT_BMP_H);

		clear_to_color(places[i].enter_trace, makecol(255,0,255));
		clear_to_color(places[i].exit_trace, makecol(255,0,255));
	
		draw_sprite_h_flip(places[i].enter_trace, places[7 - i].enter_trace,0,0);
		draw_sprite_h_flip(places[i].exit_trace, places[7 - i].exit_trace,0,0);
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
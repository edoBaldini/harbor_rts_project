#include "harbor.h"
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


void * user_task(void * arg)
{   
char scan;
int time_passed;
int pos = -1;
int ship_index = -1;
int half_num_parking = PLACE_NUMBER / 2;
int delta = 28;
int offset = 19;
int l_x = 121;
int r_x = PORT_BMP_W - l_x - delta - (half_num_parking - 1) * (offset + delta);
bool is_parked = false;
struct timespec now;
struct timespec w_time;


// Task private variables
const int id = get_task_index(arg);
	set_activation(id);

	while (!end) 
	{
		clock_gettime(CLOCK_MONOTONIC, &now);
		time_passed = time_cmp(now, w_time);

		if (mouse_b == 1)
		{
			pos = click_place(offset, delta, l_x, r_x);

			if (pos>= 0)
			{
				pthread_mutex_lock(&mutex_p);
				ship_index = places[pos].ship_id;
				pthread_mutex_unlock(&mutex_p);
				
				pthread_mutex_lock(&mutex_fleet);
				is_parked = fleet[ship_index].parking;
				pthread_mutex_unlock(&mutex_fleet);

				if (ship_index >= 0 && is_parked)
				{
					pthread_mutex_lock(&mutex_fleet);
					fleet[ship_index].parking = false;
					pthread_mutex_unlock(&mutex_fleet);

					printf("ship %d woke up\n", ship_index);
				}
			}

		}

		if (mouse_b == 2)
		{
			pos = click_place(offset, delta, l_x, r_x);

			if (pos>= 0)
			{
				pthread_mutex_lock(&mutex_p);
				ship_index = places[pos].ship_id;
				pthread_mutex_unlock(&mutex_p);

				pthread_mutex_lock(&mutex_fleet);
				is_parked = fleet[ship_index].parking;
				pthread_mutex_unlock(&mutex_fleet);

				if (ship_index >= 0 && is_parked)
				{
					pthread_mutex_lock(&mutex_fleet);
					time_add_ms(&fleet[ship_index].p_time, 20);
					printf("ship %d more 20ms, actual time %ld\n", ship_index, 
											(fleet[ship_index].p_time).tv_sec);
					pthread_mutex_unlock(&mutex_fleet);
				}
			}

		}

		scan = 0;
		if (keypressed()) 
		{
			scan = readkey() >> 8;
		}

		if (scan == KEY_ENTER && time_passed >= 0)
		{
			clock_gettime(CLOCK_MONOTONIC, &w_time);
			time_add_ms(&w_time, 2000);

			init_ship();
		}

		if (scan == KEY_SPACE)
		{

			show_routes = (show_routes) ? false : true;
		}

		if (scan == KEY_ESC)
		{
			end = true;
		}

		//terminate(ships_terminated);
		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! radar\n", id);
		}
		wait_for_activation(id);
	}

return NULL;
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

			pthread_mutex_lock(&mutex_sea);
			color = getpixel(sea, x, y);
			pthread_mutex_unlock(&mutex_sea);
			for (j = 0; j <= MAX_SHIPS; j++)
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
		
		blit(radar, screen, 0, 0,910, 0, radar->w, radar->h);

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
int ship = -1;
int i = 0;
int cur_req;
bool access_port = true;
bool access_place = true;
bool enter_trace[MAX_SHIPS] = {false};
bool exit_trace[MAX_SHIPS] = {false};
bool cur_repl;
const int id = get_task_index(arg);
	set_activation(id);

	while (!end) 
	{
		
		pthread_mutex_lock(&mutex_rr);
		cur_repl = reply_access[i];
		cur_req = request_access[i];
		pthread_mutex_unlock(&mutex_rr);

		if (cur_req == Y_PLACE)
		{
			if (access_place && !enter_trace[i])
			{	
				enter_trace[i] = assign_trace(i);

				if (enter_trace[i])
				{
					printf("ship %d PLACE assigned\n", i);
					cur_repl = true;
					access_place = false;
					access_port = true;
					ship = i;
				}
			}
		}

		if (cur_req == 1 && ship == i)
		{	
			cur_req = Y_PLACE;
			access_place = true;
			ship = -1;
		}

		if (cur_req == Y_EXIT && access_place && !exit_trace[i])
		{ 
			exit_trace[i] = assign_exit(i);
			if (exit_trace[i])
			{
				printf("ship %d exiting\n", i);
				cur_repl = true;
				access_place = false;
			}
		}

		if (cur_req == -1 )
		{
			cur_req = -2;
			printf("ship %d frees\n", i);
			cur_repl = true;
			enter_trace[i] = false;
			exit_trace[i] = false;
			free_trace(i);
			access_place = true;
		}
		
		pthread_mutex_lock(&mutex_rr);
		reply_access[i] = cur_repl;
		request_access[i] = cur_req;
		pthread_mutex_unlock(&mutex_rr);


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
BITMAP * back_sea_bmp;
BITMAP * ship_cur;
int i;
float x_cur, y_cur, g_cur;

	back_sea_bmp = create_bitmap(PORT_BMP_W, PORT_BMP_H);
	clear_bitmap(back_sea_bmp);

	port_bmp = load_bitmap("port.bmp", NULL);

	// Task private variables
	const int id = get_task_index(arg);
	set_activation(id);

	while (!end) {

		clear_to_color(sea, SEA_COLOR);
		for (i = 0; i < ships_activated; ++i)
		{
			pthread_mutex_lock(&mutex_fleet);
			x_cur = fleet[i].x;
			y_cur = fleet[i].y;
			g_cur = fleet[i].traj_grade;
			ship_cur = fleet[i].boat;
			pthread_mutex_unlock(&mutex_fleet);

			pthread_mutex_lock(&mutex_sea);
			rotate_sprite(sea, ship_cur, x_cur - (XSHIP / 2 ), y_cur, itofix(degree_fix(g_cur)+64));
			pthread_mutex_unlock(&mutex_sea);
		}
		pthread_mutex_lock(&mutex_sea);
		blit(sea, back_sea_bmp, 0, 0, 0,0,sea->w, sea->h);
		pthread_mutex_unlock(&mutex_sea);
		draw_sprite(back_sea_bmp, port_bmp, 0, 0);
		
		if (show_routes)
		{
			pthread_mutex_lock(&mutex_route);
			for (i = 0; i < ships_activated; ++i)
			{
				draw_sprite(back_sea_bmp, routes[i].trace, 0, YSHIP / 2);
			}
			pthread_mutex_unlock(&mutex_route);

		}
//	USEFUL TO VISUALIZE CHECK_FWD()
		/*for (int k = 0; k < ships_activated; ++k){
			float alpha = M_PI;
			for (int j = (YSHIP / 2); j < 70; j++)
			{
				alpha += 0.2;
				alpha = (alpha >= 2 * M_PI) ? M_PI : alpha;
				float x = fleet[k].x + j * cos(fleet[k].traj_grade);
				float y =  fleet[k].y + j * sin(fleet[k].traj_grade) + (YSHIP / 2);
 				putpixel(back_sea_bmp, x, y, 0);
 				float x1 = fleet[k].x + j * cos(alpha);
				float y1 =  fleet[k].y + j * sin(alpha) + (YSHIP / 2);
 				putpixel(back_sea_bmp, x1, y1, 0);
			}
		}*/

		putpixel(back_sea_bmp, X_PORT, Y_PORT, makecol(255,0,255));
		blit(back_sea_bmp, screen, 0,0,0,0,back_sea_bmp->w, back_sea_bmp->h); 

		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! display\n", id);
		}
		wait_for_activation(id);

	}

	for (i = 0; i < ships_activated; ++i)
		destroy_bitmap(fleet[i].boat);
		

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
	enable_hardware_cursor();
	show_mouse(screen);
	
	sea = create_bitmap(PORT_BMP_W, PORT_BMP_H);
	radar = create_bitmap(R_BMP_W, PORT_BMP_H);

	//sea_color = makecol(0,85,165);

	clear_to_color(sea, SEA_COLOR);
	clear_bitmap(radar);

	circle(radar, R_BMP_W / 2, R_BMP_H / 2, R_BMP_H / 2, makecol(255, 255, 255));
	fill_places();

	enter_trace[0] = load_bitmap("e1.bmp", NULL);
	enter_trace[1] = load_bitmap("e2.bmp", NULL);
	enter_trace[2] = load_bitmap("e3.bmp", NULL);
	
	pthread_mutex_init(&mutex_rr, NULL);
	pthread_mutex_init(&mutex_p, NULL);
	pthread_mutex_init(&mutex_fleet, NULL);
	pthread_mutex_init(&mutex_route, NULL);
	pthread_mutex_init(&mutex_sea, NULL);

	task_create(display, 50	, 69, PRIO);
	task_create(radar_task, 3, 6, PRIO);
	task_create(controller_task, PERIOD, DLINE, PRIO);
	task_create(user_task, PERIOD, DLINE, PRIO);

}

void init_ship()
{
int i;
int index = (ships_activated % 3);
bool active;

	if (ships_activated < MAX_SHIPS)
	{
		printf("ships_activated  %d  MAX_SHIPS %d\n", ships_activated, MAX_SHIPS);

		pthread_mutex_lock(&mutex_fleet);
		fleet[ships_activated].boat = create_bitmap(XSHIP, YSHIP);
		fleet[ships_activated].boat = load_bitmap("ship_c.bmp", NULL);
		fleet[ships_activated].parking = false;
		mark_label(fleet[ships_activated].boat);
		fleet[ships_activated].x = 0.0; 
		fleet[ships_activated].y = 0.0; 
		fleet[ships_activated].active = true;
		pthread_mutex_unlock(&mutex_fleet);

		pthread_mutex_lock(&mutex_route);
		routes[ships_activated].x = X_PORT;
		routes[ships_activated].y = Y_PORT;
		routes[ships_activated].trace = enter_trace[index]; 
		routes[ships_activated].odd = (index == 1);
		pthread_mutex_unlock(&mutex_route);

		ships_activated += 1;

		task_create(ship_task, PERIOD, DLINE, PRIO);
	}
	else
	{
		for (i = 0; i < MAX_SHIPS; ++i)
		{	
			pthread_mutex_lock(&mutex_fleet);
			active = fleet[i].active;
			pthread_mutex_unlock(&mutex_fleet);
			if (!active)
			{
				printf("reassigned %d\n", i);
				pthread_mutex_lock(&mutex_fleet);
				fleet[i].parking = false;
				fleet[i].x = 0.0;
				fleet[i].y = 0.0;
				fleet[i].active = true;
				pthread_mutex_unlock(&mutex_fleet);

				pthread_mutex_lock(&mutex_route);
				routes[i].x = X_PORT;
				routes[i].y = Y_PORT;
				routes[i].trace = enter_trace[i % 3]; 
				routes[i].odd = ((i % 3) == 1);
				pthread_mutex_unlock(&mutex_route);

				break;
			}
		}
	}
}

int main()
{
int i;

	if (MAX_SHIPS + AUX_THREAD > MAX_THREADS)
	{
		printf("too many ships! The max number of ships + the auxiliar thread"
		" must be lower than max number of thread (32)\n");
		return 0;
	}

	init();
	
	for (i = 0; i < AUX_THREAD; ++i)
	{
		join_spec_thread(i);

	}
	
	allegro_exit();
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
// FUNCTIONS FOR USER
//------------------------------------------------------------------------------

int click_place(int offset, int delta, int l_x, int r_x)
{
int i, space;
int half_num_parking = PLACE_NUMBER / 2;

	if (mouse_y <= Y_PLACE && mouse_y >= Y_PLACE - YSHIP)
	{
		for (i = 0; i < half_num_parking; ++i)
		{
			space = i * (offset + delta);
			if(mouse_x <= l_x + space + delta && mouse_x >= l_x + space)
				return i;

			else if (mouse_x <= r_x + space + delta && mouse_x >= r_x + space)
					return i + half_num_parking;

		}
		return -1;
	}
	else return -1;
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR CONTROLLER
//------------------------------------------------------------------------------
bool assign_trace(int ship)
{
int i = random_in_range(0, PLACE_NUMBER -1);
bool available;

	pthread_mutex_lock(&mutex_p);
	available = places[i].available;
	pthread_mutex_unlock(&mutex_p);

		if (available)
		{	
			pthread_mutex_lock(&mutex_route);
			routes[ship].trace = places[i].enter_trace;
			routes[ship].y = Y_PLACE - YSHIP;
			routes[ship].odd = (i < 4);
			pthread_mutex_unlock(&mutex_route);

			pthread_mutex_lock(&mutex_p);
			places[i].available = false;
			places[i].ship_id = ship;
			pthread_mutex_unlock(&mutex_p);
			return true;
		}

	return false;
}

bool assign_exit(int ship)
{
int i;
int ship_assigned;
	for(i = 0; i < PLACE_NUMBER; i++)
	{
		pthread_mutex_lock(&mutex_p);
		ship_assigned = places[i].ship_id;
		pthread_mutex_unlock(&mutex_p);
		if (ship_assigned == ship)
		{
			pthread_mutex_lock(&mutex_route);
			routes[ship].trace = places[i].exit_trace;
			routes[ship].y = Y_PORT;
			pthread_mutex_unlock(&mutex_route);
			return true;
		}
	}
	return false;
}

void free_trace(int ship)
{
int i;
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
	places[7].enter_trace = load_bitmap("w1.bmp", NULL);
	places[7].exit_trace = load_bitmap("x1.bmp", NULL);

	places[6].enter_trace = load_bitmap("w2.bmp", NULL);
	places[6].exit_trace = load_bitmap("x2.bmp", NULL);

	places[5].enter_trace = load_bitmap("w3.bmp", NULL);
	places[5].exit_trace = load_bitmap("x3.bmp", NULL);

	places[4].enter_trace = load_bitmap("w4.bmp", NULL);
	places[4].exit_trace = load_bitmap("x4.bmp", NULL);

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


void mark_label(BITMAP * boat)
{

	int color_assigned = 255 - (ships_activated * 10);
	rectfill(boat, 5,7, 12, 14, makecol(0,0, color_assigned));
	//printf("color assigned %d\n", makecol(0,0, color_assigned));

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
#include "harbor.h"
#include <math.h>
#include <stdio.h>
#include <pthread.h>
#include "ptask.h"

#define XWIN			1400		// width monitor
#define YWIN			700			// height monitor
#define PERIOD			10			// in ms
#define DLINE			15			// in ms
#define PRIO			10			// priority level
#define AUX_THREAD 		4
#define MAX_THREADS		12			
#define MAX_SHIPS		MAX_THREADS - AUX_THREAD			// max number of ship MUST BE LOWER THAN 30
#define FPS				200.0		
#define FRAME_PERIOD	(1 / FPS)
#define EPSILON			3.f			// guardian distance to the goal
#define	PLACE_NUMBER	8			// number of parking

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
#define MIN_P_TIME		1000		// min ship parking time, in ms
#define	MAX_P_TIME		50000		// max ship parking time, in s

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
struct place places[PLACE_NUMBER];
struct ship fleet[MAX_SHIPS];
struct route routes[MAX_SHIPS];

BITMAP * sea;
BITMAP * radar;
int sea_color;
int ships_activated = 0;
bool end = false;

int request_access[MAX_SHIPS];
bool reply_access[MAX_SHIPS];

pthread_mutex_t mutex_rr;
pthread_mutex_t mutex_p;
pthread_mutex_t mutex_fleet;
pthread_mutex_t mutex_route;
pthread_mutex_t mutex_sea;


void * user_task(void * arg)
{   
char scan;
int pos = -1;
int ship_index = -1;
int half_num_parking = PLACE_NUMBER / 2;
int delta = 28;
int offset = 19;
int l_x = 121;
int r_x = PORT_BMP_W - l_x - delta - (half_num_parking - 1) * (offset + delta);
bool is_parked = false;
// Task private variables
const int id = get_task_index(arg);
	set_activation(id);

	while (!end) 
	{
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

		if (scan == KEY_ENTER){
			init_ship();
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


void * ship_task(void * arg)
 {

int i;
int ship_id;
int last_index;
int cur_req;
int time_wakeup;
pair mytrace[X_PORT * Y_PORT];
struct timespec now;
bool mytrace_computed;
bool first_step, second_step, third_step, fourth_step;
bool cur_repl;
bool move = true;
bool wait = false;
bool termination = false;
bool is_terminated;
bool is_parked;
bool is_odd;
float x_cur, y_cur, g_cur;
BITMAP * cur_trace;

	// Task private variables
const int id = get_task_index(arg);
	set_activation(id);
	ship_id 			= id - AUX_THREAD;
	first_step			= true;
	second_step 		= false;
	third_step 			= false;
	fourth_step			= false;
	mytrace_computed 	= false;
	i 					= 0;

	while (!end && fleet[ship_id].active) 
	{
		pthread_mutex_lock(&mutex_fleet);
		x_cur = fleet[ship_id].x;
		y_cur = fleet[ship_id].y;
		g_cur = fleet[ship_id].traj_grade;
		is_terminated = fleet[ship_id].active;
		is_parked = fleet[ship_id].parking;
		clock_gettime(CLOCK_MONOTONIC, &now);
		time_wakeup = time_cmp(now, fleet[ship_id].p_time);
		pthread_mutex_unlock(&mutex_fleet);

		pthread_mutex_lock(&mutex_route);
		cur_trace = routes[ship_id].trace;
		is_odd = routes[ship_id].odd;
		pthread_mutex_unlock(&mutex_route);

		pthread_mutex_lock(&mutex_rr);
		cur_repl = reply_access[ship_id];
		cur_req = request_access[ship_id];
		pthread_mutex_unlock(&mutex_rr);

		move = (check_forward(x_cur, y_cur, g_cur)) ? false: true;

		if (first_step)
		{	
			if (!mytrace_computed)
			{
				last_index = make_array_trace(cur_trace, mytrace, 
												ship_id, is_odd, cur_req);
				mytrace_computed = true;
				i = 0;
			}

			if(check_yposition(y_cur, YGUARD_POS))
			{
				cur_req = Y_PORT;
				second_step = true;
				first_step = false;
			}

			else if (move)
			{
				follow_track_frw(ship_id, i, mytrace, last_index);
				i++;
			}

		}

		if (second_step && cur_repl)
		{
			if(check_yposition(y_cur, Y_PORT))
			{
				cur_repl = false;
				cur_req = Y_PLACE;
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

		if (third_step && cur_repl)
		{
			if (!mytrace_computed)
			{
				last_index = make_array_trace(cur_trace, mytrace, 
												ship_id, is_odd, cur_req);
				mytrace_computed = true;
				i = 0;
			}

			if(check_yposition(y_cur, Y_PLACE - YSHIP))
			{

				if (!wait)
				{	
					is_parked = true;

					pthread_mutex_lock(&mutex_fleet);
					clock_gettime(CLOCK_MONOTONIC, &fleet[ship_id].p_time);
					time_add_ms(&fleet[ship_id].p_time, random_in_range(MIN_P_TIME, MAX_P_TIME));
					pthread_mutex_unlock(&mutex_fleet);

					wait = true;
				}
				else
				{
					if (time_wakeup >= 0 || !is_parked)
					{
						is_parked = false;
						cur_repl = false;
						cur_req = Y_EXIT;
						mytrace_computed = false;
						third_step = false;
						fourth_step = true;
					}
				}
			}

			else if(move)
			{
				follow_track_frw(ship_id, i, mytrace, last_index);
				i++;
			}

		}

		if (fourth_step && cur_repl)
		{
			if (!mytrace_computed)
			{
				last_index = make_array_trace(cur_trace, mytrace,
				 								ship_id, is_odd, cur_req);
				mytrace_computed = true;
				i = 0;
			}
			if (y_cur < mytrace[0].y)
				rotate90_ship(ship_id, x_cur,Y_PLACE, mytrace[0].y + YSHIP);

			else
			{
				follow_track_frw(ship_id, i, mytrace, last_index);
				i++;
			}

			if (check_yposition(y_cur, Y_PORT - XSHIP) && 
												cur_req != -1)
			{
				cur_repl = false;
				cur_req = -1;
			}

			if (cur_req == -1 && cur_repl)
			{
				termination = exit_ship(ship_id, x_cur);
				if (termination)
					is_terminated = false;
			}
		}
		pthread_mutex_lock(&mutex_fleet);
		fleet[ship_id].active = is_terminated;
		fleet[ship_id].parking = is_parked;
		pthread_mutex_unlock(&mutex_fleet);

		pthread_mutex_lock(&mutex_rr);
		reply_access[ship_id] = cur_repl;
		request_access[ship_id] = cur_req;
		pthread_mutex_unlock(&mutex_rr);


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
int ship = -1;
int i = 0;
int cur_req;
bool access_port = true;
bool access_place = true;
bool enter_trace[MAX_SHIPS] = {false};
bool exit_trace[MAX_SHIPS] = {false};
bool cur_repl;
float cur_y;
const int id = get_task_index(arg);
	set_activation(id);

	while (!end) 
	{
		
		pthread_mutex_lock(&mutex_rr);
		cur_repl = reply_access[i];
		cur_req = request_access[i];
		pthread_mutex_unlock(&mutex_rr);

		pthread_mutex_lock(&mutex_fleet);
		cur_y = fleet[i].y;
		pthread_mutex_unlock(&mutex_fleet);

		if (cur_req == Y_PORT)
		{
			if (access_port)
			{
				printf("ship %d enters to the port\n", i);
				pthread_mutex_lock(&mutex_route);
				routes[i].x = X_PORT;
				routes[i].y = Y_PORT;
				pthread_mutex_unlock(&mutex_route);
				cur_repl = true;
				access_port = false;

			}
		}

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

		pthread_mutex_lock(&mutex_fleet);
		if (check_yposition(fleet[ship].y, Y_PLACE))
		{
			access_place = true;
			ship = -1;
		}
		pthread_mutex_unlock(&mutex_fleet);

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

		if (cur_req == -1 && check_yposition(cur_y, Y_PORT - XSHIP) &&
															!cur_repl)
		{
			printf("ship %d frees\n", i);
			cur_repl = true;

			free_trace(i);
			access_place = true;
		}
		
		pthread_mutex_lock(&mutex_rr);
		reply_access[i] = cur_repl;
		request_access[i] = cur_req;
		pthread_mutex_unlock(&mutex_rr);


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

		clear_to_color(sea, sea_color);
		for (i = 0; i < ships_activated; ++i)
		{
			pthread_mutex_lock(&mutex_fleet);
			x_cur = fleet[i].x;
			y_cur = fleet[i].y;
			g_cur = fleet[i].traj_grade;
			ship_cur = fleet[i].boat;
			pthread_mutex_unlock(&mutex_fleet);
			/*pivot_sprite(sea, fleet[i].boat, fleet[i].x, fleet[i].y, 
									fleet[i].boat-> w / 2, 0, 
									itofix(degree_fix(fleet[i].traj_grade)+64));*/
			pthread_mutex_lock(&mutex_sea);
			rotate_sprite(sea, ship_cur, x_cur - (XSHIP / 2 ), y_cur, itofix(degree_fix(g_cur)+64));
			pthread_mutex_unlock(&mutex_sea);

		}
		pthread_mutex_lock(&mutex_sea);
		blit(sea, back_sea_bmp, 0, 0, 0,0,sea->w, sea->h);
		pthread_mutex_unlock(&mutex_sea);
		draw_sprite(back_sea_bmp, port_bmp, 0, 0);
		
//	USEFUL TO VISUALIZE THE TRACKS THAT THE SHIPS ARE FOLLOWING
		//for (int k = 0; k < ships_activated; k++)
		//	if (routes[0].trace != NULL)
		//		draw_sprite(back_sea_bmp, routes[k].trace, 0,0);
		
//	USEFUL TO VISUALIZE CHECK_FWD()
		/*for (int k = 0; k < ships_activated; ++k)
			for (int j = 2; j < 70; j++)
			{
				putpixel(back_sea_bmp, fleet[k].x + j * cos(fleet[k].traj_grade), fleet[k].y + j * sin(fleet[k].traj_grade), 0);
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

	sea_color = makecol(0,85,165);

	clear_to_color(sea, sea_color);
	clear_bitmap(radar);

	circle(radar, R_BMP_W / 2, R_BMP_H / 2, R_BMP_H / 2, makecol(255, 255, 255));
	fill_places();

	pthread_mutex_init(&mutex_rr, NULL);
	pthread_mutex_init(&mutex_p, NULL);
	pthread_mutex_init(&mutex_fleet, NULL);
	pthread_mutex_init(&mutex_route, NULL);
	pthread_mutex_init(&mutex_sea, NULL);

	task_create(display, PERIOD, DLINE, PRIO);
	task_create(radar_task, 3, 6, PRIO);
	task_create(controller_task, PERIOD, DLINE, PRIO);
	task_create(user_task, PERIOD, DLINE, PRIO);

}

void init_ship()
{
int i;
int index = (ships_activated % 3);
BITMAP * enter_trace[3];

enter_trace[0] = load_bitmap("e1.bmp", NULL);
enter_trace[1] = load_bitmap("e2.bmp", NULL);
enter_trace[2] = load_bitmap("e3.bmp", NULL);

	if (ships_activated < MAX_SHIPS)
	{
		printf("ships_activated %d\n", ships_activated);

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
		routes[ships_activated].y = YGUARD_POS;
		routes[ships_activated].trace = enter_trace[index]; 
		routes[ships_activated].odd = (index == 1);
		pthread_mutex_unlock(&mutex_route);

		ships_activated += 1;

		task_create(ship_task, 20, 25, PRIO);
	}
	else
	{
		for (i = 0; i < MAX_SHIPS; ++i)
		{
			if (!fleet[i].active)
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
				routes[i].y = YGUARD_POS;
				routes[i].trace = enter_trace[index]; 
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

int make_array_trace(BITMAP * t, pair trace[PORT_BMP_W * PORT_BMP_H], int id, 
															bool odd, int req)
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

	if (req == Y_EXIT)
		reverse_array(trace, last_index);

	//routes[id].x = trace[last_index].x;
	return last_index;
}

bool check_forward(float x_cur, float y_cur, float g_cur)
{
int x, y, j;
int color;

	for (j = 2; j < 70; ++j )
	{
		x = x_cur + j * cos(g_cur);
		y = y_cur + j * sin(g_cur);
		color = getpixel(sea, x, y);

		if (color != sea_color && color != -1)
			return true; 		
	}
	
 	return false;
}

bool check_yposition(float y_ship, int y)
{
	return fabs(y_ship - y) <= EPSILON;
}

void follow_track_frw(int id, int i, pair mytrace[X_PORT * Y_PORT], 
																int last_index)
{
	if(i <= last_index)
	{
		pthread_mutex_lock(&mutex_fleet);
		fleet[id].x = mytrace[i].x;
		fleet[id].y = mytrace[i].y;

		if (last_index > i + 60)
			fleet[id].traj_grade = degree_rect(fleet[id].x, fleet[id].y, 
						mytrace[i + 20].x, mytrace[i + 20].y);
		else 
			fleet[id].traj_grade = fleet[id].traj_grade;
		pthread_mutex_unlock(&mutex_fleet);
	}
}

void rotate90_ship(int id, float x_cur, int y1, int y2)
{
float aux = 5000 / PERIOD;
	
	if (x_cur > X_PORT)
	{
		pthread_mutex_lock(&mutex_fleet);
		fleet[id].y += (y2 - y1) / aux;
		fleet[id].traj_grade -= (M_PI / 2) / aux;
		pthread_mutex_unlock(&mutex_fleet);
	}
	else
	{
		pthread_mutex_lock(&mutex_fleet);
		fleet[id].y += (y2 - y1) / aux;
		fleet[id].traj_grade += (M_PI / 2) / aux;
		pthread_mutex_unlock(&mutex_fleet);
	}
}

bool exit_ship(int id, float x_cur)
{
float aux = 1000 / PERIOD;
	if(x_cur < X_PORT)
	{
		if (x_cur > -YSHIP)
		{
			pthread_mutex_lock(&mutex_fleet);
			fleet[id].x -= YSHIP / aux;
			pthread_mutex_unlock(&mutex_fleet);

			return false;
		}
		else return true;
	}
	else
	{	if (x_cur < PORT_BMP_W + YSHIP)
		{	
			pthread_mutex_lock(&mutex_fleet);
			fleet[id].x += YSHIP/ aux;
			pthread_mutex_unlock(&mutex_fleet);
			return false;
		}
		else return true;
	}
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

void terminate(int ships_terminated[MAX_SHIPS])
{
int i, j;
	for (i = 0; i < ships_activated; ++i)
	{
		if (!fleet[i].active && !ships_terminated[i])
		{
			j = join_spec_thread(i + AUX_THREAD);
			ships_terminated[i] = true;
			printf("joined i %d status %d\n", i + AUX_THREAD, j);
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
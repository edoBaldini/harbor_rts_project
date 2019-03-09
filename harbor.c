#include "common.h"
#include <math.h>
#include <stdio.h>
#include <pthread.h>
#include "ptask.h"
#include "user.h"
#include "ship.h"
//-----------------------------------------------------------------------------
//	MACRO related to the radar
//------------------------------------------------------------------------------
#define R_BMP_W			451			//	width radar
#define R_BMP_H			451			//	height radar
#define RMAX			450			//	max radius 
#define RSTEP			1			//	step of the radius
#define RMIN			0			//	min radius
#define XRAD			450			//	x radar center<
#define YRAD			450			//	y radar center
//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
BITMAP * sea;						//	bitmap in which are drawn ships	
BITMAP * radar;						//	bitmap in which are drawn radar points
BITMAP * enter_trace[3];			//	array of the ingress trace

//	one place will be assigned to one route for a certain period of time
struct place places[PLACE_NUMBER];
struct ship fleet[MAX_SHIPS];		//	fleet of ship
struct route routes[MAX_SHIPS];		//	routes[i] related with fleet[i]

// 	positions required by ships. request_access[i] related with fleet[i]
int request_access[MAX_SHIPS];
int ships_activated = 0;			//	number of ship activated

//	ship allowed to move. reply_access[i] related with fleet[i]
bool reply_access[MAX_SHIPS];
bool end = false;					// 	true when user presses ESC
bool show_routes = false;			// 	true when user presses SPACE BAR 

pthread_mutex_t mutex_fleet;		//	for fleet
pthread_mutex_t mutex_route;		//	for routes
pthread_mutex_t mutex_rr;			//	for request_acces & reply_access
pthread_mutex_t mutex_p;			//	for places
pthread_mutex_t mutex_sea;			//	for sea 
pthread_mutex_t mutex_end;			//	for end
pthread_mutex_t mutex_s_route;		//	for show_route
pthread_mutex_t mutex_s_activated;	//	for ship_activated
static pthread_mutex_t mutex_radar;	//	for radar

//------------------------------------------------------------------------------
//	RADAR FUNCTIONS
//------------------------------------------------------------------------------

//	checks if the given color is equal to the blue of the ship
bool check_ship(int color);

//	Given an angle alpha. It put a pixel on the radar if detects a ship color
void radar_one_line(float alpha);

//------------------------------------------------------------------------------
//	CONTROLLER FUNCTIONS
//------------------------------------------------------------------------------

//	set the attributes of the route linked to the given ship with the input data
void fill_trace(int ship, int i, BITMAP * trace);

//------------------------------------------------------------------------------
//	it returns true if assigns a place's trace to the given ship. 
//	Otherwise all the places are occupied and it returns false.
//------------------------------------------------------------------------------
bool assign_trace(int ship);

//	it assigns the exit trace associated to the ship's parking and returns true
bool assign_exit(int ship);

//	It manages the ships that trying to arrive to Y_PORT.
int access_port(int i, int ship_to_port, int ship_to_place);

//	It manages the ships that trying to arrive to Y_PLACE.
int access_place(int i, int ship_to_place);

//	makes parking available that was occupied by the given ship id
void free_trace(int ship);

//------------------------------------------------------------------------------
//	DISPLAY FUNCTIONS
//------------------------------------------------------------------------------

//	draw on the screen the the ships activated in their current position
void view_ships(BITMAP * boat);

//	draw on the screen the the routes of the ship activated
void view_routes();

//	converts the provided grade in a fixed point 16.16 grade
fixed degree_fix(float grade);

//------------------------------------------------------------------------------
//	AUXILIAR FUNCTIONS
//------------------------------------------------------------------------------

//	initializes the environment and creates the 4 auxiliar tasks
void init(void);

//	load the bitmaps related to the places and initializes its attributes
void fill_places();

//------------------------------------------------------------------------------
//	TASK FUNCTIONS
//------------------------------------------------------------------------------

//	The user_task replies to the user inputs. 
void * user_task(void * arg);

//	The radar detects the ships over the map
void * radar_task(void * arg);

//	The controller manage the flow of the ships in the harbor.
void * controller_task(void *arg);

//	The display manage all the screen, and in the end will destroy the bitamps
void * display(void *arg);

int main()
{
	init();	//	initializes the environment

	wait_tasks();
	allegro_exit();
	return 0;
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR RADAR
//------------------------------------------------------------------------------

//	checks if the given color is equal to the blue of the ship
bool check_ship(int color)
{
	int  blue = makecol(0,0, 255);
	return color == blue;
}

//------------------------------------------------------------------------------
//	Given an angle alpha. It put a pixel on the radar if detects a ship color 
//	between its position and a range.
//------------------------------------------------------------------------------
void radar_one_line(float alpha)
{
bool found = false;					//	true if detects a ship
float x, y;							//	coordinates that must to be checked
float scale = ((float) (XRAD) / (PORT_BMP_W));	//	scale from map to radar
int d, color;
int white = makecol(255, 255, 255);	//	color of the point of the radar

	for (d = RMIN; d < RMAX; d += RSTEP)	//	range of the radar line
	{
		x = XRAD + d * cos(alpha);
		y = YRAD - d * sin(alpha);

		pthread_mutex_lock(&mutex_sea);
		color = getpixel(sea, x, y);
		pthread_mutex_unlock(&mutex_sea);

		found = check_ship(color);	//	true if color is equal to ship's blue
		if (found)
		{
			pthread_mutex_lock(&mutex_radar);
			putpixel(radar, (x * scale), (y * scale), white);
			pthread_mutex_unlock(&mutex_radar);
		}
	}
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR CONTROLLER
//------------------------------------------------------------------------------

//	set the attributes of the route linked to the given ship with the input data
void fill_trace(int ship_id, int i, BITMAP * trace)
{
	pthread_mutex_lock(&mutex_route);
	routes[ship_id].trace = trace;
	routes[ship_id].flip = (i < 4);
	routes[ship_id].index = -1;
	pthread_mutex_unlock(&mutex_route);
}

//------------------------------------------------------------------------------
//	it returns true if assigns a place's trace to the given ship. 
//	Otherwise all the places are occupied and it returns false.
//------------------------------------------------------------------------------
bool assign_trace(int ship)
{
int i = random_in_range(0, PLACE_NUMBER -1);	//	the place is chosen randomly
bool available;
BITMAP * enter_trace;

	pthread_mutex_lock(&mutex_p);
	available = places[i].available;
	pthread_mutex_unlock(&mutex_p);

		if (available)			//	checks if the selected place is available
		{	
			pthread_mutex_lock(&mutex_p);
			places[i].available = false;			//	update the availability
			places[i].ship_id = ship;
			enter_trace = places[i].enter_trace;	//	assigns place's trace
			pthread_mutex_unlock(&mutex_p);

			fill_trace(ship, i, enter_trace);		//	set the new route

			return true;
		}

	return false;
}

//	it assigns the exit trace associated to the ship's parking and returns true
bool assign_exit(int ship)
{
int i;
int ship_assigned;
BITMAP * exit_trace;

	//	scan the parking places finding the place related to the given ship
	for(i = 0; i < PLACE_NUMBER; i++)
	{
		pthread_mutex_lock(&mutex_p);
		ship_assigned = places[i].ship_id;
		exit_trace = places[i].exit_trace;
		pthread_mutex_unlock(&mutex_p);
		if (ship_assigned == ship)
		{
			//	when the ship's place is found update the route
			fill_trace(ship, i, exit_trace);
			return true;
		}
	}
	return false;
}

//------------------------------------------------------------------------------
//	It manages the ships that trying to arrive to Y_PORT.
//	Given the state of the port and of the place, it allows to the given ship to
//	reach the Y_PORT only if no one ship has already this permission
//------------------------------------------------------------------------------
int access_port(int id, int ship_to_port, int ship_to_place)
{
int cur_req;	//	current request_access value of the given ship
bool cur_repl;	//	current reply_access value of the given ship
	
	pthread_mutex_lock(&mutex_rr);
	cur_repl = reply_access[id];
	cur_req = request_access[id];
	pthread_mutex_unlock(&mutex_rr);

	//	if the state of the port is free and the ship has requested the port
	if (ship_to_port == -1 && cur_req == Y_PORT)
	{
		cur_repl = true;	//	the ship can enter
		ship_to_port = id;	//	memorize the ship id that has the permission
		printf("ship %d enters to the port\n", id);
	}

	//	if the ship has both the authorization for the port and for the place
	else if (ship_to_port == id && ship_to_place == id)
	{
		ship_to_port = -1;	//	the state of the port return free
	}
	
	pthread_mutex_lock(&mutex_rr);
	reply_access[id] = cur_repl;
	pthread_mutex_unlock(&mutex_rr);
	
	return ship_to_port;
}

//------------------------------------------------------------------------------
//	It manages the ships that trying to arrive to Y_PLACE.
//	Given the state of the place, it allows to the given ship to reach the
//	Y_PLACE only if no one ship has already this permission
//------------------------------------------------------------------------------
int access_place(int id, int ship_to_place)
{
int cur_req;	//	current request_access value of the given ship
bool cur_repl;	//	current reply_access value of the given ship
	
	pthread_mutex_lock(&mutex_rr);
	cur_repl = reply_access[id];
	cur_req = request_access[id];
	pthread_mutex_unlock(&mutex_rr);

	if (ship_to_place == -1)	//	if the state of the place is free
	{
		//	if the ship has asked for the place but has not the permission
		if (cur_req == Y_PLACE && !cur_repl) {

			cur_repl = assign_trace(id);	//	tries to assign a place

			//	if cur_repl is true, the place state is blocked by id
			ship_to_place = (cur_repl) ? id : ship_to_place;

			if (cur_repl) {
				printf("ship %d PLACE assigned\n", id);
			}
		}

		if (cur_req == Y_EXIT) {
			assign_exit(id);		//	assigns the exit trace to the ship
			ship_to_place = id;	//	blocked the place state
			cur_repl = true;	//	assigns the permission to exit
			printf("ship %d exiting\n", id);
		}
	}
	
	//	if the ship is exited o is parked, frees the place state
	if (ship_to_place == id && (cur_req == 1 || cur_req == -1))
	{
		ship_to_place = -1;
	}

	pthread_mutex_lock(&mutex_rr);
	reply_access[id] = cur_repl;
	pthread_mutex_unlock(&mutex_rr);

	return ship_to_place;
}

//	makes parking available that was occupied by the given ship id
void free_trace(int id)
{
int i;
int cur_req = get_req(id);	//	current request_access value of the given ship
	
	if (cur_req == -1)
	{
		for (i = 0; i < PLACE_NUMBER; i++)
		{
			pthread_mutex_lock(&mutex_p);
			if (places[i].ship_id == id)
			{
				places[i].available = true;	//	the place is available
				places[i].ship_id = -1;		//	the place has not ship
			}
			pthread_mutex_unlock(&mutex_p);
		}
		printf("ship %d frees\n", id);
		cur_req = -2;					//	the ship has terminated its cycle
	}

	pthread_mutex_lock(&mutex_rr);
	request_access[id] = cur_req ;
	pthread_mutex_unlock(&mutex_rr); 
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR DISPLAY
//------------------------------------------------------------------------------

//	draw on the screen the the ships activated in their current position
void view_ships(BITMAP * boat)
{
int i;
int cur_s_activated = get_s_activated();	//	current number of ships live
ship cur_ship;
float offset = (XSHIP / 2 );				//	offset to center the ship
float actual_x;								//	actual position on x axis
fixed actual_a;								//	actual ship angle in fixed point

	for (i = 0; i < cur_s_activated; ++i)
	{
		pthread_mutex_lock(&mutex_fleet);
		cur_ship = fleet[i];
		pthread_mutex_unlock(&mutex_fleet);

		actual_x = cur_ship.x - offset;
		actual_a = degree_fix(cur_ship.traj_grade);

		pthread_mutex_lock(&mutex_sea);
		rotate_sprite(sea, boat, actual_x ,cur_ship.y, actual_a);
		pthread_mutex_unlock(&mutex_sea);
	}

	pthread_mutex_lock(&mutex_sea);
	blit(sea, screen, 0, 0, 0,0,sea->w, sea->h);
	pthread_mutex_unlock(&mutex_sea);
}

//	draw on the screen the the routes of the ship activated
void view_routes()
{
int i, j;
int entrance[3] = {-1, -1, -1};	//	ships id that print the enter traces
int cur_s_activated = get_s_activated();	//	current number of ships live
ship cur_ship;
bool parked;					//	true if a ship has reached a parking place

	for (i = 0; i < cur_s_activated; ++i)
	{
		pthread_mutex_lock(&mutex_fleet);
		cur_ship = fleet[i];
		pthread_mutex_unlock(&mutex_fleet);
		parked = check_position(cur_ship.y, Y_PLACE - YSHIP);
		
		pthread_mutex_lock(&mutex_route);
		if (cur_ship.y >= Y_PORT)	//	check if the ship is below the Y_PORT
		{
			//	if the ship has the first trace and no one has printed it yet
			if (cur_ship.x < X_PORT && entrance[0] == -1) {	
				entrance[0] = i;	//	notes that first trace must be printed
			}

			//	if the ship has the second trace and no one has printed it yet
			if (cur_ship.x == X_PORT && entrance[1] == -1) {
				entrance[1] = i;	//	notes that second trace must be printed
			}
			
			//	if the ship has the third trace and no one has printed it yet
			if (cur_ship.x > X_PORT && entrance[2] == -1) {
				entrance[2] = i;	//	notes that third trace must be printed
			}

			for (j = 0; j < 3; ++j)	//	print the enter traces
			{
				if (i == entrance[j])
				{
					draw_sprite(screen, routes[i].trace, 0, YSHIP / 2);	
				}
			}
		}

		//	if the ship is above the port, has a trace yet and it is not parked
		if (cur_ship.y < Y_PORT && routes[i].trace != NULL && !parked) {
			draw_sprite(screen, routes[i].trace, 0, YSHIP / 2);	
		}
		pthread_mutex_unlock(&mutex_route);
	}
}

//	converts the provided grade in a fixed point 16.16 grade
fixed degree_fix(float grade)
{
	int degree = grade * 360 / (2 * M_PI);	//	from radiants to degree
	int new_degree = degree * 256 / 360;	//	from circle in 360 to 256 degree
	int right_angle = 64;
	fixed fix_grade =  itofix(new_degree + right_angle);
	return fix_grade;						//	angle in fixed point 16.16
}

//------------------------------------------------------------------------------
// FUNCTIONS FOR AUXILIAR
//------------------------------------------------------------------------------

//	initializes the environment and creates the 4 auxiliar tasks
void init(void)
{
	allegro_init();
	install_keyboard();
	install_mouse();

	set_color_depth(16);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN,0,0);
	enable_hardware_cursor();
	show_mouse(screen);
	
	//	create the bitamp in which will be drawn and checked the ships
	sea = create_bitmap(PORT_BMP_W, PORT_BMP_H);
	radar = create_bitmap(R_BMP_W, PORT_BMP_H);

	clear_to_color(sea, SEA_COLOR);
	clear_bitmap(radar);

	circle(radar, R_BMP_W / 2, R_BMP_H / 2, R_BMP_H / 2, makecol(255, 255, 255));
	fill_places();	//	initializes the attributes related to the parking places

	enter_trace[0] = load_bitmap("e1_c.bmp", NULL);
	enter_trace[1] = load_bitmap("e3_c.bmp", NULL);
	enter_trace[2] = load_bitmap("e2_c.bmp", NULL);

	srand(time(0));	//	needed to generate then random	values

	pthread_mutex_init(&mutex_rr, NULL);
	pthread_mutex_init(&mutex_p, NULL);
	pthread_mutex_init(&mutex_fleet, NULL);
	pthread_mutex_init(&mutex_route, NULL);
	pthread_mutex_init(&mutex_sea, NULL);
	pthread_mutex_init(&mutex_radar, NULL);
	pthread_mutex_init(&mutex_end, NULL);
	pthread_mutex_init(&mutex_s_route, NULL);
	pthread_mutex_init(&mutex_s_activated, NULL);

	task_create(display, PERIOD	, DLINE, PRIO);
	task_create(radar_task, PERIOD, DLINE, PRIO);
	task_create(controller_task, PERIOD, DLINE, PRIO);
	task_create(user_task, PERIOD, DLINE, PRIO);
}

//	load the bitmaps related to the places and initializes its attributes
void fill_places()
{
int i, j;
int h = PLACE_NUMBER / 2;				//	will be the left places
	for (j = 0; j < PLACE_NUMBER; ++j)
	{
		places[j].ship_id = -1;			//	no ships are assigned to a place
		places[j].available = true;		//	the place is available
	}
	places[7].enter_trace = load_bitmap("w1_c.bmp", NULL);
	places[7].exit_trace = load_bitmap("x1_c.bmp", NULL);

	places[6].enter_trace = load_bitmap("w2_c.bmp", NULL);
	places[6].exit_trace = load_bitmap("x2_c.bmp", NULL);

	places[5].enter_trace = load_bitmap("w3_c.bmp", NULL);
	places[5].exit_trace = load_bitmap("x3_c.bmp", NULL);

	places[4].enter_trace = load_bitmap("w4_c.bmp", NULL);
	places[4].exit_trace = load_bitmap("x4_c.bmp", NULL);

	for (i = 0; i < h; ++i)
	{
		places[i].enter_trace = create_bitmap(PORT_BMP_W, PORT_BMP_H);
		places[i].exit_trace = create_bitmap(PORT_BMP_W, PORT_BMP_H);

		clear_to_color(places[i].enter_trace, makecol(255,0,255));
		clear_to_color(places[i].exit_trace, makecol(255,0,255));
	
		draw_sprite_h_flip(places[i].enter_trace,
								places[PLACE_NUMBER -1 - i].enter_trace, 1, 0);
		draw_sprite_h_flip(places[i].exit_trace, 
								places[PLACE_NUMBER -1 - i].exit_trace, 0, 0);
	}	
}

//------------------------------------------------------------------------------
//	TASK FUNCTIONS
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//	The user_task replies to the user input. 
//	The user can interact via mouse and keyboard.
//
//	Mouse button pressed over a parked ship:
//	-	left invokes woke_up()
//	-	right invokes add_parking_time()
//	
//	The keyboard interaction is managed thanks to button_pressed()
//------------------------------------------------------------------------------
void * user_task(void * arg)
{   
bool c_end = false;					//	current value of the global variable end
const int id = get_task_index(arg);	//	task id
	set_activation(id);

	while (!c_end) 
	{		
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		switch (mouse_b)				//	if the user click over a parked ship 
		{
			case 1:
					woke_up();			//	frees from its parking
					break;

			case 2:
					add_parking_time();	//	adds time to the parking time
					break;
		}

		button_pressed();				//	manages the keyboard interactions 

		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! radar\n", id);
		}
		wait_for_activation(id);
	}
return NULL;
}

//------------------------------------------------------------------------------
//	The radar detects the ships over the map
//	It scan the sea bitmap searching ships, they are labeled with a special
//	color. All the pixels with that color are reported, scaled, in the radar.
//------------------------------------------------------------------------------
void * radar_task(void * arg)
{   
bool c_end = false;					//	current value of the global variable end
float a = 0.f;						//	angle of the radar radius
float alpha;						//	angle in radiants

const int id = get_task_index(arg);
	set_activation(id);

	while (!c_end) 
	{   
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		alpha = a * M_PI / 180.f;   // 	from degree to radiants
		radar_one_line(alpha);		//	scan over the radius with angle alpha
		a += 1.5; 					//	constant increase of the angle

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

//------------------------------------------------------------------------------
//	The controller manage the flow of the ships in the harbor.
//	It decides when a ship can enter in the harbor, it assigns the places and 
//	decides when a ship can exit.
//	It can read the requests of the ship and reply if a ship can move or not
//------------------------------------------------------------------------------
void * controller_task(void *arg)
{
int i = 0;
int ship_to_port = -1;		//	state of the route to the port, -1 means free
int ship_to_place = -1;		//	state of the route to the parking, -1 means free
bool c_end = false;			//	current value of the global variable end
const int id = get_task_index(arg);
	set_activation(id);

	while (!c_end) 
	{
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		//	it will maintain the id of the ship allowed to reach the port
		ship_to_port = access_port(i, ship_to_port, ship_to_place);

		//	it will maintain the id of the ship allowed to reach the parking
		ship_to_place = access_place(i, ship_to_place);
		
		free_trace(i);	//	frees the parking places does not occupied
		
		i = (i < MAX_SHIPS) ? i + 1 : 0;

		if (deadline_miss(id))
		{   
			printf("%d) deadline missed! ship\n", id);
		}
		wait_for_activation(id);

	}
	return NULL;
}

//------------------------------------------------------------------------------
//	The display manage all the screen, and in the end will destroy the bitamps
//	It shows the harbor, ships, routes and radar.
//------------------------------------------------------------------------------
void * display(void *arg)
{
BITMAP * port_bmp;					//	harbor bitmap
BITMAP * boat;						//	ship bitmap
int i;
int radar_pos = PORT_BMP_W + 10;	//	radar spaced 10 from the map
int white = makecol(255, 255, 255);	//	color of the radar
bool c_end = false;					//	current value of the global variable end

	port_bmp = load_bitmap("port.bmp", NULL);
	boat = load_bitmap("ship_c.bmp", NULL);

	const int id = get_task_index(arg);
	set_activation(id);

	while (!c_end) {
		pthread_mutex_lock(&mutex_end);
		c_end = end;
		pthread_mutex_unlock(&mutex_end);

		clear_to_color(sea, SEA_COLOR);

		view_ships(boat);			//	shows on the screen the ships

		draw_sprite(screen, port_bmp, 0, 0);
		
		if (show_routes)
		{
			view_routes();			//	shows on the screen the routes
		}

		putpixel(screen, X_PORT, Y_PORT, makecol(255,0,255));
		putpixel(screen, X_PORT, YGUARD_POS, makecol(255,0,255));

		pthread_mutex_lock(&mutex_radar);
		circle(radar, XRAD, YRAD, RMAX, white);
		blit(radar, screen, 0, 0,radar_pos, 0, radar->w, radar->h);
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

	for (i = 0; i < ENTER_NUMBER; ++i)
	{
		destroy_bitmap(enter_trace[i]);
	}
	
	destroy_bitmap(port_bmp);
	destroy_bitmap(boat);
	destroy_bitmap(sea);

	return NULL;
}
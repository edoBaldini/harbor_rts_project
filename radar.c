#include <allegro.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include "ptask.h"

#define XWIN			1400		// width monitor
#define YWIN            680        	// height monitor
#define	XSHIP			18			// width dimension of the ship	
#define YSHIP			54			// height dimension of the ship
#define PERIOD          40          // in ms
#define DLINE           60
#define PRIO            10
#define MAX_SHIPS       12          // max number of ship MUST BE LOWER THAN 30
#define MAX_THREADS     32
#define FPS             60.0
#define FRAME_PERIOD    (1 / FPS)
#define EPSILON         2.f        // guardian distance to the goal

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
#define XRAD            450			//x center of the radar = center of the port
#define YRAD            450			//y center of the radar = center of the port

#define YGUARD_POS      600
#define XPORT           450			//x position of the door port
#define YPORT           505			//y postizion of the doow port
#define VEL             100

#define PORT_BMP_W      900
#define PORT_BMP_H      900
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

typedef struct route
{
	BITMAP * trace;
	float x, y; 
}route;


struct ship fleet[MAX_SHIPS];
struct route routes[MAX_SHIPS];
//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
BITMAP * sea;
BITMAP * radar;
BITMAP * t1;
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
    float y_m = y_target - y_target;
    
    return sqrtf((x_m * x_m) + (y_m * y_m));
}

float degree_rect(float x1, float y1, float x2, float y2)
{	
    float angular_coefficient = (x1 == x2) ? x1 : ((y2 - y1) / (x2 - x1));
    float degree = atanf(angular_coefficient);
    return (x2 <= x1) ? degree + M_PI  : degree + 2 * M_PI;

}

float actual_vel(float x, float y, 
                    float xtarget_pos, float ytarget_pos, float s_vel)
{
float velx;
float vely;
float vel;

    velx = 2 * (xtarget_pos - x);
    vely = 2 * (ytarget_pos - y);
    vel = (sqrtf(velx * velx + vely * vely));
    return (fabs(vel) > s_vel) ? s_vel : vel;
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

 void follow_track(int i)
{	
float a = 0;
float d;
float alpha;
int color;
float x,y;
	
	while (a < 180)
	{
		alpha = a * M_PI / 180.f;
		for(d = 60; d > 0; d -= 6)
		{
			x = fleet[i].x + d * cosf(alpha);
        	y = fleet[i].y - d * sinf(alpha);
        	color = getpixel(t1, x, y);
        	if (color != 63519)
        	{
        		fleet[i].traj_grade = degree_rect(fleet[i].x, fleet[i].y, x,y);
        		fleet[i].vel = 0.05;
        		fleet[i].x = xlinear_movement(fleet[i].x, x, fleet[i].vel, fleet[i].traj_grade);
        		fleet[i].y = ylinear_movement(fleet[i].y, y, fleet[i].vel, fleet[i].traj_grade);
        		d = 0;
        	}

		}
		a += 0.02;

	}
}

 void * ship_task(void * arg)
 {
bool need_stop = true; // MUST BE CHANGED!!!!
int i,y,x;
int ship_id;
ship * myship;
route * myroute;
    // Task private variables
    const int id = get_task_index(arg);
    set_activation(id);
    ship_id = id - aux_thread;
    myship = &fleet[ship_id];
    myroute = &routes[ship_id];
    while (!end) 
    {
    	if (myroute-> trace == NULL)
    	{
    		myship-> traj_grade = degree_rect(myship-> x, myship-> y, 
                                                myroute-> x, myroute-> y);
        	myship-> x = xlinear_movement(myship-> x, myroute-> x, myship-> vel, 
        										myship-> traj_grade);
   			myship-> y = ylinear_movement(myship-> y, myroute-> y, myship-> vel, 
        										myship-> traj_grade);
    	}

    	else 
    		follow_track(ship_id);
    	/*else{
    		x = 0;
    		y = 0;
    		for(i = 20; i > 0; --i)
    		{	
    			if (getpixel(t1, myship-> x, myship-> y- i) == 0)
    				y++;
    			if (getpixel(t1, myship-> x - i, myship-> y) == 0)
    				x++;
    		}

    		if (x != 0 || y != 0){
    			myship-> traj_grade = degree_rect(myship-> x, myship-> y, x, y);
    			if (y!=0)
    		   		myship-> y = ylinear_movement(myship-> y, y, myship-> vel, 
        										myship-> traj_grade);
    			if (x != 0)
    				myship-> x = xlinear_movement(myship-> x, x, myship-> vel, 
        										myship-> traj_grade);

    			printf("angolo %f\n", myship->traj_grade);
    		}*/
    		/*if (getpixel(t1, myship-> x - 30, myship-> y-10) != 63519)
    		{
    			printf("mi sposto a sinistra\n");
    			myship-> y = ylinear_movement(myship-> y, myship-> y - 3, myship-> vel, 
        										myship-> traj_grade);
    			myship-> x = xlinear_movement(myship-> x, myship-> x - 3, myship-> vel, 
        										myship-> traj_grade);
    		}
    		else {
    			printf("vado su\n");
    			myship-> y = ylinear_movement(myship-> y, myship-> y - 3, myship-> vel, 
        										myship-> traj_grade);
    		}*/
    	//}

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
	return fabs(fleet[id].x - routes[id].x) <= EPSILON &&
			fabs(fleet[id].y - routes[id].y) <= EPSILON;
}

 bool try_access_port(bool access_port)
 {
 int j;
 	if (access_port)
 	{
 	for (j = 0; j < ships_activated; ++j)
        {
        	if (fabs(YGUARD_POS - fleet[j].y) <= EPSILON && access_port)
        	{
        		routes[j].y = YPORT;
        		routes[j].x = XPORT;
        		return false;
        	}
        }
    }
    return access_port;
 }

void * controller_task(void *arg)
{
int i, j;
bool access_port = true;
bool access_route = false;
bool free_places[13];

const int id = get_task_index(arg);
	set_activation(id);

	for (j = 0; j < 13; j++)
		free_places[j] = true;

    while (!end) {
        access_port = try_access_port(access_port);
        if (!access_port)
        {
        	for(i = 0; i  < ships_activated; ++i)
        	{
        		if (check_spec_position(i, XPORT, YPORT)){
        			fleet[i].traj_grade = (- M_PI / 2);
        			routes[i].trace = t1;
        		}

        	}
        }



       /* if (id_ship >= 0 && check_position(id_ship))
        {
        	access_port = false;
        	routes[id_ship].x = XPORT + 200;
        	routes[id_ship].y = YPORT - 200;

        }*/
        if (deadline_miss(id))
        {   
            printf("%d) deadline missed! ship\n", id);
        }
        wait_for_activation(id);

    }

    return NULL;
}

float degree_fix(float grade)
{
	int new_grade = (grade > 0 ? grade : (2 * M_PI + grade)) * 360 / (2 * M_PI); //from radiants to degree 360
    return (new_grade * 256 / 360);
}
 void * display(void *arg)
 {
BITMAP * port_bmp;
BITMAP * back_sea_bmp;
int i;
    // Task private variables
    const int id = get_task_index(arg);
    set_activation(id);

    back_sea_bmp = create_bitmap(PORT_BMP_W, PORT_BMP_H);
    sea = create_bitmap(PORT_BMP_W, PORT_BMP_H);
    radar = create_bitmap(R_BMP_W, PORT_BMP_H);

    sea_color = makecol(0,85,165);

    clear_bitmap(back_sea_bmp);
    clear_to_color(sea, sea_color);
    clear_bitmap(radar);

    port_bmp = load_bitmap("port.bmp", NULL);
    t1 = load_bitmap("t1.bmp", NULL);

    circle(radar, R_BMP_W / 2, R_BMP_H / 2, R_BMP_H / 2, makecol(255, 255, 255));

    while (!end) {

        clear_to_color(sea, sea_color);
        for (i = 0; i < ships_activated; i++ )
            pivot_sprite(sea, fleet[i].boat, fleet[i].x, fleet[i].y, 
                            fleet[i].boat-> w / 2, 0, ftofix(degree_fix(fleet[i].traj_grade)+64));

        blit(sea, back_sea_bmp, 0, 0, 0,0,sea->w, sea->h);
        draw_sprite(back_sea_bmp, port_bmp, 0, 0);
        draw_sprite(back_sea_bmp, t1, 0,0);
        circle(back_sea_bmp, XPORT, YPORT, 10, 0);
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
        fleet[ships_activated].x = (ships_activated * 55 + 150) % PORT_BMP_W;
        fleet[ships_activated].y = random_in_range(PORT_BMP_H, YWIN);
        fleet[ships_activated].vel = VEL;

        routes[ships_activated].trace = NULL;
        routes[ships_activated].x = fleet[ships_activated].x;
        routes[ships_activated].y = YGUARD_POS;
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
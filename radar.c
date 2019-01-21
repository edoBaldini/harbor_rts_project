#include <allegro.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include "ptask.h"

#define XWIN            1400        // width monitor
#define YWIN            730        	// height monitor
#define	XSHIP			18			// width dimension of the ship	
#define YSHIP			54			// height dimension of the ship
#define PERIOD          40          // in ms
#define DLINE           60
#define PRIO            10

#define FPS             60.0
#define FRAME_PERIOD    (1 / FPS)
#define EPSILON         0.f        // guardian distance to the goal

//-----------------------------------------------------------------------------
// GLOBAL CONSTANTS related to the radar
//------------------------------------------------------------------------------
#define RMAX            450
#define ARES            360
#define RSTEP           10
#define RMIN            0
#define XRAD            450			//x center of the radar = center of the port
#define YRAD            450			//y center of the radar = center of the port

#define XPORT           450			//x position of the door port
#define YPORT           680			//y postizion of the doow port
#define VEL             60

#define true 			1
#define false 			0

//-----------------------------------------------------------------------------
// GLOBAL STRUCTURE
//------------------------------------------------------------------------------
typedef int bool;

typedef struct SHIP
{
    float x, y;
    float width, height;
    float vel;
    float bow_grade;
    float traj_grade; 
} SHIP;

//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
BITMAP * port_bmp;
BITMAP * sea;
BITMAP  * ship;
SHIP    titanic;
int sea_color;

//------------------------------------------------------------------------------
// FUNCTIONS FOR RADAR
//------------------------------------------------------------------------------

void * radar_task(void * arg)
{   
	//int radar_bmp = create_bitmap()

// Task private variables
    const int id = ptask_id(arg);
    ptask_activate(id);

    int a = 0;
    while (!key[KEY_ESC]) 
    {
        float alpha;
		int d = 0;
		int x, y;
		int r, g, b, color;
		alpha = a * M_PI / 180.f;   // from degree to radiants
    	for (d = RMIN; d < RMAX; d += RSTEP)
    	{
        	x = XRAD + d * cos(alpha);
        	y = YRAD - d * sin(alpha);
        	color = getpixel(sea, x, y);

        	if (color != sea_color && color != 0){
        		printf("ciao, x %d, y %d\n", x, y); 

        	}

			circlefill(sea, x, y, 1, 0);

        	//r = getr(color);
        	//g = getg(color);
        	//b = getb(color);
        	//printf("x %d, y %d\n", x, y); 


   		}
// from the formula L = pi * r *a / 180 I can guarantee that, the circumference
// arc len is less than the width ship in this way the ships will be always seen
        a += 2;	
        if (a == 360)
            a = 0;
        if (ptask_deadline_miss(id))
        {   
            printf("%d) deadline missed!\n", id);
        }
        ptask_wait_for_activation(id);
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

float distance_vector(float x_start, float y_start, float x_target, float y_target)
{
    float x_m = x_target - x_start;
    float y_m = y_target - y_target;
    
    return sqrtf((x_m * x_m) + (y_m * y_m));
}

float degree_rect(float x1, float y1, float x2, float y2)
{
    float angular_coefficient   = (x1 == x2) ? x1 : ((y2 - y1) / (x2 - x1));
    float degree                = atan(angular_coefficient);
    return (x2 <= x1) ? degree + M_PI  : degree + 2 * M_PI;
    //return degree;
}

void linear_movement(float xtarget_pos,float ytarget_pos, bool reg_vel)
{   
    float velx;
    float vely;
    float vel;

    titanic.traj_grade = degree_rect(titanic.x, titanic.y, xtarget_pos, ytarget_pos);

    if (reg_vel)
    {
        velx = 2 * (xtarget_pos - titanic.x) / 3;
        vely = 2 * (ytarget_pos - titanic.y) / 3;
        vel = (sqrtf(velx * velx + vely * vely));
        titanic.vel = (fabs(vel) > VEL) ? VEL : vel;
    }

    titanic.x = titanic.x + (titanic.vel * cos(titanic.traj_grade) / PERIOD);
    titanic.y = titanic.y + (titanic.vel * sin(titanic.traj_grade) / PERIOD);
 }

 void * ship_task(void * arg)
 {
    float route[] = {   
                        XPORT, YPORT    
                    };
    bool reached[] = {
                        false, false,
                    };
    // Task private variables
    const int id = ptask_id(arg);
    ptask_activate(id);
    int i = 0;
    int len_route = 2;
    while (!key[KEY_ESC]) {
        
        if (i < len_route)
        {   
           if (!reached[i])
            {  
                bool need_stop = (i + 1) == (len_route - 1); 
                linear_movement(route[i], route[i+1], need_stop);
                reached[i] =    (fabs(route[i] - titanic.x) <= EPSILON && 
                                 fabs(route[i+1] - titanic.y) <= EPSILON) ? true : false;
            }
            else
                i = i + 2;
        }

        if (ptask_deadline_miss(id))
        {   
            printf("%d) deadline missed!\n", id);
        }
        ptask_wait_for_activation(id);

    }

    return NULL;
 }
int main()
{
	allegro_init();
	install_keyboard();
	set_color_depth(16);
	set_gfx_mode(GFX_AUTODETECT_WINDOWED, XWIN, YWIN,0,0);
	
	port_bmp = load_bitmap("port.bmp", NULL);
	sea	= create_bitmap(900, 900);
	sea_color = makecol(0,85,165);
	clear_to_color(sea, sea_color);

	ship = create_bitmap(XSHIP, YSHIP);
	ship = load_bitmap("ship.bmp", NULL);
	BITMAP *buffer = NULL;

	buffer = create_bitmap(XWIN, YWIN);

    titanic.x       =   random_in_range(0, 900);
    titanic.y       =   random_in_range(900, 680);
    titanic.vel     =   VEL;
    titanic.width   =   ship->w;    
    titanic.height  =   ship->h;

	ptask_create(radar_task, 5, 10, PRIO);
	ptask_create(ship_task, PERIOD, DLINE, PRIO);
	while(!key[KEY_ESC])
	{
		clear_to_color(sea, sea_color);
		draw_sprite(sea, ship, (titanic.x - titanic.width /2) , titanic.y);
		blit(sea, buffer, 0, 0, 0, 0, sea->w, sea->h);
		draw_sprite(buffer, port_bmp, 0, 0);
		circle(buffer, XPORT, YPORT, 10, 2);
		circle(buffer,1150, 225, 225, makecol(255,255,255));
		blit(buffer, screen, 0,0,0,0,XWIN, YWIN);

	}

	destroy_bitmap(port_bmp);
	destroy_bitmap(buffer);

    return 0;
}
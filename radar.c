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
#define MAX_SHIPS       12          // max number of ship
#define FPS             60.0
#define FRAME_PERIOD    (1 / FPS)
#define EPSILON         0.f        // guardian distance to the goal

#define true            1
#define false           0
//-----------------------------------------------------------------------------
// GLOBAL CONSTANTS related to the radar
//------------------------------------------------------------------------------
#define RADAR_BMP_W     451
#define RADAR_BMP_H     451

#define RMAX            450
#define ARES            360
#define RSTEP           1
#define RMIN            0
#define XRAD            450			//x center of the radar = center of the port
#define YRAD            450			//y center of the radar = center of the port

#define XPORT           450			//x position of the door port
#define YPORT           680			//y postizion of the doow port
#define VEL             60

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

struct ship fleet[MAX_SHIPS];

//------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------
BITMAP * sea;
BITMAP * radar;
BITMAP * t1;
int sea_color;
int ships_activated = 0;
bool end = false;

//------------------------------------------------------------------------------
// FUNCTIONS FOR RADAR
//------------------------------------------------------------------------------

void * radar_task(void * arg)
{   
	//int radar_bmp = create_bitmap()

// Task private variables
    const int id = ptask_id(arg);
    ptask_activate(id);
    bool flag = true;
    float a = 0;
    while (!end) 
    {   
        float alpha;
		int d = 0;
		int x, y;
		int r, g, b, color;
		alpha = a * M_PI / 180.f;   // from degree to radiants
        for( int j = 0; j < 2; j++){
    	for (d = RMIN; d < RMAX; d += RSTEP)
    	{
        	x = XRAD + d * cos(alpha);
        	y = YRAD - d * sin(alpha);
        	color = getpixel(sea, x, y);
            if (color == makecol(0,0,255) && flag){
                circlefill(radar, (x / 2), (y / 2), 3, makecol(255,255,255));
                flag = false;
        	}
   		}
// from the formula L = pi * r *a / 180 I can guarantee that, the circumference
// arc len is less than the width label in this way the ships will be always seen
        a += 1;	

        if (a == 360.0){
            a = 0.0;
            flag = true;
            clear_bitmap(radar);
            circle(radar, RADAR_BMP_W / 2, RADAR_BMP_H / 2, RADAR_BMP_H / 2, makecol(255, 255, 255));

        }
    
        if (ptask_deadline_miss(id))
        {   
            printf("%d) deadline missed! radar\n", id);
        }
        ptask_wait_for_activation(id);
    }
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
    float angular_coefficient = (x1 == x2) ? x1 : ((y2 - y1) / (x2 - x1));
    float degree = atan(angular_coefficient);
    return (x2 <= x1) ? degree + M_PI  : degree + 2 * M_PI;
}

float actual_vel(float x, float y, float xtarget_pos, float ytarget_pos, float s_vel)
{
float velx;
float vely;
float vel;

    velx = 2 * (xtarget_pos - x) / 3;
    vely = 2 * (ytarget_pos - y) / 3;
    vel = (sqrtf(velx * velx + vely * vely));
    return (fabs(vel) > s_vel) ? VEL : vel;
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

 void follow_track(ship s)
{
    if (getpixel(t1, s.x + 1, s.y - 1 ) == 0)
    {
        //titanic.x += 1;
        //titanic.y -= 1;
        s.traj_grade = degree_rect(s.x, s.y, s.x +1 , s.y - 1);
        s.x = xlinear_movement(s.x, s.x + 1, s.vel, s.traj_grade);
        s.y = ylinear_movement(s.y, s.y - 1, s.vel, s.traj_grade);

    }

    if (getpixel(t1, s.x - 1, s.y - 1 ) == 0)
    {
        //titanic.x -= 1;
        //titanic.y -= 1;
        s.traj_grade = degree_rect(s.x, s.y, s.x - 1 , s.y - 1);
        s.x = xlinear_movement(s.x, s.x - 1, s.vel, s.traj_grade);
        s.y = ylinear_movement(s.y, s.y - 1, s.vel, s.traj_grade);    
    }

    
}

 void * ship_task(void * arg)
 {
bool need_stop = true; // MUST BE CHANGED!!!!

    printf("ciao task\n");
    // Task private variables
    const int id = ptask_id(arg);
    ptask_activate(id);
    int i = 0;
    while (!end) {
        for (i = 0; i < ships_activated; i++)
        {
            fleet[i].traj_grade = degree_rect(fleet[i].x, fleet[i].y, XPORT, YPORT);

            if (need_stop)
                fleet[i].vel = actual_vel(fleet[i].x, fleet[i].y, XPORT, YPORT, fleet[i].vel);

           fleet[i].x = xlinear_movement(fleet[i].x, XPORT, fleet[i].vel, fleet[i].traj_grade);
           fleet[i].y = ylinear_movement(fleet[i].y, YPORT, fleet[i].vel, fleet[i].traj_grade);

        
        
        fleet[i].traj_grade = 0;
        }
        //follow_track();

        if (ptask_deadline_miss(id))
        {   
            printf("%d) deadline missed! ship\n", id);
        }
        ptask_wait_for_activation(id);

    }

    return NULL;
 }

 void * display(void *arg)
 {
BITMAP * port_bmp;
BITMAP * back_sea_bmp;
int i;
    // Task private variables
    const int id = ptask_id(arg);
    ptask_activate(id);

    back_sea_bmp = create_bitmap(PORT_BMP_W, PORT_BMP_H);
    sea = create_bitmap(PORT_BMP_W, PORT_BMP_H);
    radar = create_bitmap(RADAR_BMP_W, PORT_BMP_H);

    sea_color = makecol(0,85,165);

    clear_bitmap(back_sea_bmp);
    clear_to_color(sea, sea_color);
    clear_bitmap(radar);

    port_bmp = load_bitmap("port.bmp", NULL);
    t1 = load_bitmap("t1.bmp", NULL);

    circle(radar, RADAR_BMP_W / 2, RADAR_BMP_H / 2, RADAR_BMP_H / 2, makecol(255, 255, 255));

    while (!end) {

        clear_to_color(sea, sea_color);
        for (i = 0; i < ships_activated; i++ )
            pivot_sprite(sea, fleet[i].boat, fleet[i].x, fleet[i].y, fleet[i].boat-> w / 2, 0, itofix(0));
        blit(sea, back_sea_bmp, 0, 0, 0,0,sea->w, sea->h);
        draw_sprite(back_sea_bmp, port_bmp, 0, 0);
        circle(back_sea_bmp, XPORT, YPORT, 10, 0);
        blit(back_sea_bmp, screen, 0,0,0,0,back_sea_bmp->w, back_sea_bmp->h); //valuta se mettere XWIN, YWIN
        blit(radar, screen, 0, 0,910, 0, radar->w, radar->h);

        if (ptask_deadline_miss(id))
        {   
            printf("%d) deadline missed! display\n", id);
        }
        ptask_wait_for_activation(id);

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
    
    ptask_create(display, PERIOD, DLINE, PRIO);
    ptask_create(radar_task, 3, 6, PRIO);
 }

void mark_label(BITMAP * boat)
{

    int color_assigned = 255 - ships_activated;
    rectfill(boat, 5,7, 12, 14, makecol(0,0, color_assigned));
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
        fleet[ships_activated].x = random_in_range(0, PORT_BMP_W);
        fleet[ships_activated].y =   random_in_range(PORT_BMP_H, YWIN);
        fleet[ships_activated].vel     =   VEL;
        ships_activated += 1;

        printf("random x %f y %f\n", fleet[ships_activated -1 ].x, fleet[ships_activated -1].y);
        ptask_create(ship_task, PERIOD, DLINE, PRIO);
    }
 }

int main()
{

char scan;
int i;

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
    ptask_wait_tasks();

    return 0;
}
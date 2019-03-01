#ifndef USER_H
#define USER_H

//------------------------------------------------------------------------------
//	USER FUNCTIONS
//------------------------------------------------------------------------------
struct timespec botton_pressed(struct timespec pressed);
int find_parked();
void woke_up();
void add_parking_time();
void initialize_ship(int i);
void init_ship();
int click_place(int offset, int delta, int l_x, int r_x);

#endif
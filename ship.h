#ifndef SHIP_H
#define SHIP_H

//------------------------------------------------------------------------------
//	SHIP FUNCTIONS
//------------------------------------------------------------------------------
int compute_mytrace(int ship_id, bool is_odd, pair mytrace[X_PORT * Y_PORT], 
																	BITMAP * cur_trace, int obj);
void reverse_array(pair trace[], int last_index);
void make_array_trace(BITMAP * t, pair trace[], int id, bool odd, int req);
bool check_forward(float x_cur, float y_cur, float g_cur);
bool check_position(float y_ship, int y);
int follow_track_frw(int id, int i, pair mytrace[], int last_index, bool move, 
															BITMAP * cur_trace);
void rotate90_ship(int id, float x_cur, int y1, int y2);
bool exit_ship(int id, float x_cur);
float distance_vector (float x1, float y1, float x2, float y2);
int find_index(pair mytrace[X_PORT * Y_PORT], int posix);

#endif
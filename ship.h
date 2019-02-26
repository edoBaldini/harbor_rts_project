#ifndef SHIP_H
#define SHIP_H

//------------------------------------------------------------------------------
//	SHIP FUNCTIONS
//------------------------------------------------------------------------------
int compute_mytrace(int ship_id, bool is_odd, triple mytrace[X_PORT * Y_PORT], 
																	BITMAP * cur_trace, int obj);
bool check_forward(float x_cur, float y_cur, float g_cur);
int follow_track_frw(int id, int i, triple mytrace[], int last_index, bool move);
void reverse_array(triple trace[], int last_index);
void make_array_trace(BITMAP * t, triple trace[], bool odd, int req);
void rotate90_ship(int id, float x_cur, int y1, int y2);
bool exit_ship(int id, float x_cur);
float distance_vector (float x1, float y1, float x2, float y2);
int find_index(triple mytrace[X_PORT * Y_PORT], int posix);
float min(float x, float y);
float max(float x, float y);
#endif
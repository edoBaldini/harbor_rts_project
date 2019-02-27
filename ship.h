#ifndef SHIP_H
#define SHIP_H

//------------------------------------------------------------------------------
//	SHIP FUNCTIONS
//------------------------------------------------------------------------------
void compute_mytrace(int ship_id, triple mytrace[X_PORT * Y_PORT], int obj);
bool check_forward(float x_cur, float y_cur, float g_cur);
void follow_track_frw(int id, triple mytrace[], bool move);
void reverse_array(triple trace[], int last_index);
void make_array_trace(BITMAP * t, triple trace[], bool odd, int req);
void rotate90_ship(int id, float x_cur, int y1, int y2);
bool exit_ship(int id, float x_cur);
float distance_vector (float x1, float y1, float x2, float y2);
int find_index(triple mytrace[X_PORT * Y_PORT], int posix);
void update_rr(int id, int repl, int req);
bool get_repl(int ship_id);
bool get_req(int ship_id);
int update_state(int id, int new_state, int obj, int last_index);
#endif
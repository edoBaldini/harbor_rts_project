#include "common.h"
#include <math.h>
#include <pthread.h>

//	calulate a random integer in the specified interval
int random_in_range(int min_x, int max_x)
{
	return rand() % (max_x + 1 - min_x) + min_x;
}

//	check if the two position differs for a tolerant space epsilon
bool check_position(float y_ship, int y)
{
	return fabs(y_ship - y) <= EPSILON;
}

//	update safely the global variable ships_activated with the given new value
void update_s_activated(int new)
{
	pthread_mutex_lock(&mutex_s_activated);
	ships_activated = new;
	pthread_mutex_unlock(&mutex_s_activated);
}

//	get safe way the value of the global variable ships_activated
int get_s_activated()
{
int n_ships;	
	
	pthread_mutex_lock(&mutex_s_activated);
	n_ships = ships_activated;
	pthread_mutex_unlock(&mutex_s_activated);

	return n_ships;
}

//	Get reply_access value associated to the given ship_id safely
bool get_repl(int ship_id)
{
bool repl;
	pthread_mutex_lock(&mutex_rr);
	repl = reply_access[ship_id];
	pthread_mutex_unlock(&mutex_rr);
	return repl;
}

//	Get request_access value associated to the given ship_id safely
int get_req(int ship_id)
{
int req;
	pthread_mutex_lock(&mutex_rr);
	req = request_access[ship_id];
	pthread_mutex_unlock(&mutex_rr);
	return req;
}
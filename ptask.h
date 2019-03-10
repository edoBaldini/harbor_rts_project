#ifndef PTASK_H
#define PTASK_H

#include <time.h>

//  It copies the given time ts in the given time td
void time_copy(struct timespec *td, struct timespec ts);

//  It adds to the given time t the value ms expressed in milliseconds
void time_add_ms(struct timespec *t, int ms);

//------------------------------------------------------------------------------
//  This function compares the given time t1 and t2. It returns:
//  0   if t1 == t2
//  1   if t1 > t2
//  -1  if t1 < t2
//------------------------------------------------------------------------------
int time_cmp(struct timespec t1, struct timespec t2);

//------------------------------------------------------------------------------
//  It creates a new thread with the given attributes imposing a scheduling RR.
//  Return 0 on success otherwhise an error number.
//------------------------------------------------------------------------------
int task_create(void * (*task_handler)(void *), int period, int drel, int prio);

//  Retrieves the task index stored in tp->id
int get_task_index(void * arg);

// It computes the next activation time and the absolute deadline of the task.
void set_activation(int id);

//------------------------------------------------------------------------------
//  If the thread execution overcome its deadline, a deadline miss occur and 
//  it increments the value of dmiss, returns 1, otherwise returns 0.
//------------------------------------------------------------------------------
int deadline_miss(int id);

//------------------------------------------------------------------------------
//  Suspends the calling thread until the next activation and then, 
//  updates activation time and deadline.
//------------------------------------------------------------------------------
void wait_for_activation(int id);

//  It waites the termination of all the activated threads.
void wait_tasks();

#endif

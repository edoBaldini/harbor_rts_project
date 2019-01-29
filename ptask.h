#ifndef PTASK_H
#define PTASK_H

#include <time.h>

void time_add_ms(struct timespec *t, int ms);
int time_cmp(struct timespec t1, struct timespec t2);
int task_create(void * (*task_handler)(void *), int period, int deadline,
                 int priority);

int get_task_index(void * arg);
void set_activation(int id);
int deadline_miss(int id);
void wait_for_activation(int id);
void wait_tasks();


#endif

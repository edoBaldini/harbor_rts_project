#ifndef PTASK_H
#define PTASK_H


int task_create(void * (*task_handler)(void *), int period, int deadline,
                 int priority);

int get_task_index(void * arg);
void set_activation(int id);
int deadline_miss(int id);
void wait_for_activation(int id);
void wait_tasks();

#endif

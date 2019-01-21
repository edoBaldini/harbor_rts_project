#ifndef PTASK_H
#define PTASK_H

#define PTASK_ERROR -1
#define PTASK_DEADLINE_MISS 1

/*
Basic task_handler implementation

void * task_handler(void * arg)
{

    // Task private variables

    const int id = ptask_id(arg);
    ptask_activate(id);

    while (true) {

        // Task business code

        if (ptask_deadline_miss(id)) {
            printf("%d) deadline missed!\n", id);
        }
        ptask_wait_for_activation(id);
    }

    // Task private variables cleanup

    return NULL;
}
*/

int ptask_create(void * (*task_handler)(void *),
                 const int period,
                 const int deadline,
                 const int priority);
int ptask_id(const void *arg);
void ptask_activate(const int id);
int ptask_deadline_miss(const int id);
void ptask_wait_for_activation(const int id);
void ptask_wait_tasks();

#endif

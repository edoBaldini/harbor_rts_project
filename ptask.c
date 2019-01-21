#include "ptask.h"
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include "time_utils.h"

#define MAX_TASKS (32)


struct task_par {
    int id;             // task argument
    long wcet;          // in microseconds
    int period;         // in milliseconds
    int deadline;       // relative (ms)
    int priority;       // in [0,99]
    int deadline_miss;  // no. of misses
    struct timespec at; // next activ. time
    struct timespec dl; // abs. deadline
};

struct task_par tp[MAX_TASKS];
pthread_t tid[MAX_TASKS];
size_t taskCount = 0;


int ptask_create(void * (*task_handler)(void *),
                 const int period,
                 const int deadline,
                 const int priority)
{
    pthread_attr_t task_attributes;
    struct sched_param task_sched_params;
    int ret;

    int id = taskCount++;
    assert(id < MAX_TASKS);

    tp[id].id = id;
    tp[id].period = period;
    tp[id].deadline = deadline;
    tp[id].priority = priority;
    tp[id].deadline_miss = 0;

    pthread_attr_init(&task_attributes);
    pthread_attr_setinheritsched(&task_attributes, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&task_attributes, SCHED_RR);
    task_sched_params.sched_priority = tp[id].priority;
    pthread_attr_setschedparam(&task_attributes, &task_sched_params);
    ret = pthread_create(&tid[id], &task_attributes, task_handler, (void *)(&tp[id]));

    if (ret != 0) return PTASK_ERROR;

    return id;
}

int ptask_id(const void * arg)
{
    struct task_par * tp = (struct task_par *)arg;
    return tp->id;
}

void ptask_activate(const int id)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    time_copy(&(tp[id].at), now);
    time_copy(&(tp[id].dl), now);
    time_add_ms(&(tp[id].at), tp[id].period);
    time_add_ms(&(tp[id].dl), tp[id].deadline);
}

int ptask_deadline_miss(const int id)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (time_cmp(now, tp[id].dl) > 0) {
        tp[id].deadline_miss++;
        return PTASK_DEADLINE_MISS;
    }
    return 0;
}

void ptask_wait_for_activation(const int id)
{
    clock_nanosleep_abstime(&(tp[id].at));
    time_add_ms(&(tp[id].at), tp[id].period);
    time_add_ms(&(tp[id].dl), tp[id].deadline);
}

void ptask_wait_tasks()
{
    for (int i = 0; i < taskCount; ++i) {
        pthread_join(tid[i], NULL);
    }
}

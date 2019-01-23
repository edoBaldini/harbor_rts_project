#include "ptask.h"
#include <pthread.h>
#include <sched.h>
#include <assert.h>

#define NT (32)

struct task_par {
    int id;             // task argument
    long wcet;          // in microseconds
    int period;         // in milliseconds
    int deadline;       // relative (ms)
    int priority;       // in [0,99]
    int dmiss;          // n. of misses
    struct timespec at; // next activ. time
    struct timespec dl; // abs. deadline
};

struct task_par tp[NT];
pthread_t tid[NT];
size_t task_counter = 0;


void time_copy(struct timespec *td, struct timespec ts)
{
    td->tv_sec = ts.tv_sec;
    td->tv_nsec = ts.tv_nsec;
}

void time_add_ms(struct timespec *t, int ms)
{
    t->tv_sec += ms/1000;
    t->tv_nsec += (ms%1000)*1000000;

    if (t->tv_nsec > 1000000000) {
        t->tv_nsec -= 1000000000;
        t->tv_sec += 1;
    }
}

int time_cmp(struct timespec t1, struct timespec t2)
{
    if (t1.tv_sec > t2.tv_sec) return 1;
    if (t1.tv_sec < t2.tv_sec) return -1;
    if (t1.tv_nsec > t2.tv_nsec) return 1;
    if (t1.tv_nsec < t2.tv_nsec) return -1;
    return 0;
}

int task_create(void * (*task_handler)(void *), int period, int drel,
                    int prio)
{
    pthread_attr_t task_attributes;
    struct sched_param task_sched_params;
    int ret;

    int id = task_counter;
    task_counter += 1;
    assert(id < NT);

    tp[id].id = id;
    tp[id].period = period;
    tp[id].deadline = derel;
    tp[id].priority = prio;
    tp[id].dmiss = 0;   //return 0 on success otherwhise an error number

    pthread_attr_init(&task_attributes);
    pthread_attr_setinheritsched(&task_attributes, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&task_attributes, SCHED_RR);
    task_sched_params.sched_priority = tp[id].priority;
    pthread_attr_setschedparam(&task_attributes, &task_sched_params);
    ret = pthread_create(&tid[id], &task_attributes, task_handler, (void *)(&tp[id]));

    return ret;
}

int get_task_index(void * arg)
{
struct task_par * tp;
    tp = (struct task_par *)arg;
    return tp->id;
}

void set_activation(const int id)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    time_copy(&(tp[id].at), t);
    time_copy(&(tp[id].dl), t);
    time_add_ms(&(tp[id].at), tp[id].period);
    time_add_ms(&(tp[id].dl), tp[id].deadline);
}

int deadline_miss(const int i)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    if (time_cmp(t, tp[i].dl) > 0) {
        tp[i].dmiss++;
        return 1;
    }
    return 0;
}

void wait_for_activation(int i)
{
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp[i].at), NULL);
    time_add_ms(&(tp[i].at), tp[i].period);
    time_add_ms(&(tp[i].dl), tp[i].deadline);
}

void wait_tasks()
{
    for (int i = 0; i < task_counter; ++i) {
        pthread_join(tid[i], NULL);
    }
}

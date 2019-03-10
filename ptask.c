#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include "ptask.h"

#define NT (128)            //  number of tasks

struct task_par {
    int id;                 //  task argument
    long wcet;              //  in microseconds
    int period;             //  in milliseconds
    int deadline;           //  relative (ms)
    int priority;           //  in [0,99]
    int dmiss;              //  n. of misses
    struct timespec at;     //  next activ. time
    struct timespec dl;     //  abs. deadline
};

struct task_par tp[NT];     //  task parameters
pthread_t tid[NT];          //  threads id
size_t task_counter = 0;    //  takes track of the activated threads number

//  It copies the given time ts in the given time td
void time_copy(struct timespec *td, struct timespec ts)
{
    td->tv_sec = ts.tv_sec;
    td->tv_nsec = ts.tv_nsec;
}

//  It adds to the given time t the value ms expressed in milliseconds
void time_add_ms(struct timespec *t, int ms)
{
    t->tv_sec += ms/1000;
    t->tv_nsec += (ms%1000)*1000000;

    if (t->tv_nsec > 1000000000) {
        t->tv_nsec -= 1000000000;
        t->tv_sec += 1;
    }
}

//------------------------------------------------------------------------------
//  This function compares the given time t1 and t2. It returns:
//  0   if t1 == t2
//  1   if t1 > t2
//  -1  if t1 < t2
//------------------------------------------------------------------------------
int time_cmp(struct timespec t1, struct timespec t2)
{
    if (t1.tv_sec > t2.tv_sec) return 1;
    if (t1.tv_sec < t2.tv_sec) return -1;
    if (t1.tv_nsec > t2.tv_nsec) return 1;
    if (t1.tv_nsec < t2.tv_nsec) return -1;
    return 0;
}

//------------------------------------------------------------------------------
//  It creates a new thread with the given attributes imposing a scheduling RR.
//  Return 0 on success otherwhise an error number.
//------------------------------------------------------------------------------
int task_create(void * (*task_handler)(void *), int period, int drel, int prio)
{
    pthread_attr_t task_attributes;
    struct sched_param task_sched_params;
    int ret;

    int id = task_counter;  //  assigns the task id
    task_counter += 1;      //  update the counter
    assert(id < NT);

    //  initializes the parameters for the new task with the provided data
    tp[id].id = id;         
    tp[id].period = period;
    tp[id].deadline = drel;
    tp[id].priority = prio;
    tp[id].dmiss = 0;       

    pthread_attr_init(&task_attributes);
    pthread_attr_setinheritsched(&task_attributes, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&task_attributes, SCHED_RR);
    task_sched_params.sched_priority = tp[id].priority;
    pthread_attr_setschedparam(&task_attributes, &task_sched_params);
    
    ret = pthread_create(&tid[id], &task_attributes, task_handler, 
                                                            (void *)(&tp[id]));

    return ret;
}

//  Retrieves the task index stored in tp->id
int get_task_index(void * arg)
{
struct task_par * tp;
    tp = (struct task_par *)arg;
    return tp->id;
}

// It computes the next activation time and the absolute deadline of the task.
void set_activation(const int id)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    time_copy(&(tp[id].at), t);
    time_copy(&(tp[id].dl), t);
    time_add_ms(&(tp[id].at), tp[id].period);
    time_add_ms(&(tp[id].dl), tp[id].deadline);
}

//------------------------------------------------------------------------------
//  If the thread execution overcome its deadline, a deadline miss occur and 
//  it increments the value of dmiss, returns 1, otherwise returns 0.
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
//  Suspends the calling thread until the next activation and then, 
//  updates activation time and deadline.
//------------------------------------------------------------------------------
void wait_for_activation(int i)
{
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp[i].at), NULL);
    time_add_ms(&(tp[i].at), tp[i].period);
    time_add_ms(&(tp[i].dl), tp[i].period);
}

//  It waites the termination of all the activated threads.
void wait_tasks()
{
    for (int i = 0; i < task_counter; ++i) {
        pthread_join(tid[i], NULL);
    }
}
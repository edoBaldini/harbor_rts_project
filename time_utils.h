#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <time.h>

#define TIME_GIGA (1000000000)


static inline void time_copy(struct timespec * td,
                             const struct timespec ts)
{
    td->tv_sec  = ts.tv_sec;
    td->tv_nsec = ts.tv_nsec;
}

static inline void time_mono_diff(struct timespec * t2,
                                  const struct timespec * t1)
{
    t2->tv_sec = t1->tv_sec - t2->tv_sec;
    t2->tv_nsec = t1->tv_nsec - t2->tv_nsec;
    if (t2->tv_sec < 0) {
        t2->tv_sec = 0;
        t2->tv_nsec = 0;
    } else if (t2->tv_nsec < 0) {
        if (t2->tv_sec == 0) {
            t2->tv_sec = 0;
            t2->tv_nsec = 0;
        } else {
            t2->tv_sec = t2->tv_sec - 1;
            t2->tv_nsec = t2->tv_nsec + TIME_GIGA;
        }
    }
}

static inline void time_add_ms(struct timespec * t,
                               const int ms)
{
    t->tv_sec += ms / 1000;
    t->tv_nsec += (ms % 1000) * 1000000;
    if (t->tv_nsec > TIME_GIGA) {
        t->tv_nsec -= TIME_GIGA;
        t->tv_sec += 1;
    }
}

static inline int time_cmp(const struct timespec t1,
                           const struct timespec t2)
{
    if (t1.tv_sec > t2.tv_sec) return 1;
    if (t1.tv_sec < t2.tv_sec) return -1;
    if (t1.tv_nsec > t2.tv_nsec) return 1;
    if (t1.tv_nsec < t2.tv_nsec) return -1;
    return 0;
}

#ifdef __MACH__
/* emulate clock_nanosleep for CLOCK_MONOTONIC and TIMER_ABSTIME */
static inline int clock_nanosleep_abstime(const struct timespec * req)
{
    struct timespec ts_delta;
    int retval = clock_gettime(CLOCK_MONOTONIC, &ts_delta);
    if (retval == 0) {
        time_mono_diff(&ts_delta, req);
        retval = nanosleep(&ts_delta, NULL);
    }
    return retval;
}
#else /* POSIX */
    /* clock_nanosleep for CLOCK_MONOTONIC and TIMER_ABSTIME */
    #define clock_nanosleep_abstime(req) \
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,(req), NULL)
#endif

#endif

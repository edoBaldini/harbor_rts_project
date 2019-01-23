#ifndef QUEUE_H
#define QUEUE_H

#include <stdlib.h>
#include <pthread.h>

#define QUEUE_SUCCESS  0
#define QUEUE_EOF      (void *)0xFFFFFFFF


#define QUEUE_SIZE 512

typedef struct queue {
    void * items[QUEUE_SIZE];
    size_t head;
    size_t tail;
    pthread_mutex_t mux;
} Queue;


int queue_create(Queue * q);
int queue_lock(Queue * q);
int queue_unlock(Queue * q);
int is_empty_unsafe(Queue * q);
int is_empty(Queue * q);
int push(Queue * q, void * data);
void * pop(Queue * q);
int destroy(Queue * q, void(*item_free)(void *));

#endif
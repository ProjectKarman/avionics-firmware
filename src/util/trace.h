/*
 * trace.h
 *
 * Created: 11/14/2015 3:03:39 PM
 *  Author: Nigil Lee
 */ 


#ifndef TRACE_H_
#define TRACE_H_

#include <stdint.h>


#define traceTASK_SWITCHED_IN() task_in((int)pxCurrentTCB->pxTaskTag)
#define traceTASK_SWITCHED_OUT() task_out((int)pxCurrentTCB->pxTaskTag)

void trace_init(void);
void task_in(uint8_t task);
void task_out(uint8_t task);


#endif /* TRACE_H_ */
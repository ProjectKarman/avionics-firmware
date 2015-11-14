/*
 * trace.c
 *
 * Created: 11/14/2015 3:03:23 PM
 *  Author: Nigil Lee
 */ 

#include "trace.h"
#include <stdint.h>
#include "asf.h"
#include "freertos.h"
#include "queue.h"

void trace_init(void) {
  ioport_set_pin_dir(DEBUG_0, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(DEBUG_1, IOPORT_DIR_OUTPUT);
}

void task_in(uint8_t task) {
  if(task == 1) {
    ioport_set_pin_high(DEBUG_0);
  }
}

void task_out(uint8_t task) {
  if(task == 1) {
    ioport_set_pin_low(DEBUG_0);
  }
}
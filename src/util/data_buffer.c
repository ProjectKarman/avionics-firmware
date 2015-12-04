#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "data_buffer.h"

// local variables
#define BUFFER_SIZE	256

// local functions


// task functions

void data_buffer_init(sensor_data_t *tranceiver_buffer)	{
  memset(tranceiver_buffer, 0, sizeof(sensor_data_t));
}

void data_buffer_add(sensor_data_t *tranceiver_buffer, sensor_data_t node) {
  // send to transceiver
		
  // save for reference (?)
  tranceiver_buffer->time_stamp = node.time_stamp;
		
  // save for reference (?)
  tranceiver_buffer->time_stamp = node.time_stamp;
}
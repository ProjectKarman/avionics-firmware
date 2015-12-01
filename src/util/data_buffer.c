#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "data_buffer.h"

// local variables
#define BUFFER_SIZE	256

// local functions
// transceiver_queue_t tranceiver_buffer

static void tranceiver_queue_reinitialize(transceiver_queue_t tranceiver_buffer);

// task functions

void data_buffer_init(transceiver_queue_t *tranceiver_buffer, reference_node_t *current_sensor_readings)	{
	memset(tranceiver_buffer, 0, sizeof(transceiver_queue_t));
	memset(current_sensor_readings, 0, sizeof(reference_node_t));
}

void data_buffer_add(transceiver_queue_t *tranceiver_buffer, reference_node_t *current_sensor_readings, sensor_data_t node) {
	if(tranceiver_buffer->current_size == BUFFER_SIZE) {
		// send to transceiver
		
		// save for reference (?)
		current_sensor_readings->time_stamp = node.time_stamp;
		
		// reinitialize queue
		tranceiver_buffer->current_size = 0;
	}
	else {
		tranceiver_buffer->sensor_task_data_buffer[tranceiver_buffer->current_size] = node;
		tranceiver_buffer->current_size++;
		
		// save for reference (?)
		current_sensor_readings->time_stamp = node.time_stamp;
	}
}
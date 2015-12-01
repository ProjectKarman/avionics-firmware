
#ifndef DATA_BUFFER_H_

#include<stdio.h>
#define BUFFER_SIZE	256

typedef struct {
	//double p[3];//position
	//double v[3];//velocity
	//double a[3];//acceleration
	//double radius;
	//double mass;
	uint32_t time_stamp;
} reference_node_t;

typedef struct {
	//double p[3];//position
	//double v[3];//velocity
	//double a[3];//acceleration
	//double radius;
	//double mass;
	uint32_t time_stamp;
}sensor_data_t;


typedef struct {
	uint8_t current_size;
	sensor_data_t sensor_task_data_buffer[BUFFER_SIZE];
} transceiver_queue_t;


void data_buffer_init(transceiver_queue_t *tranceiver_buffer, reference_node_t *current_sensor_readings);
void data_buffer_add(transceiver_queue_t *tranceiver_buffer, reference_node_t *current_sensor_readings, sensor_data_t node);

#endif /* DATA_BUFFER_H_ */

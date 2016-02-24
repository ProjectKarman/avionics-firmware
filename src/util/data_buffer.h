
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
}sensor_data_t;

void data_buffer_init(sensor_data_t *tranceiver_struct);
void data_buffer_add(sensor_data_t *tranceiver_struct, sensor_data_t node);

#endif /* DATA_BUFFER_H_ */

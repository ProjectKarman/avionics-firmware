/*
 * sensor.h
 *
 * Created: 11/19/2015 8:59:37 PM
 *  Author: Nigil Lee
 */ 


#ifndef SENSOR_H_
#define SENSOR_H_

void sensor_start_task(void);

typedef void (*twi_interface_initialize_t)(void);

#endif /* SENSOR_H_ */
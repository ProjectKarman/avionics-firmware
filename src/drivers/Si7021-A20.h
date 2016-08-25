/*
Si7021-A20.c
Author: Matan Silver
Created: 2/2/2016
*/

#ifndef TEMP_HUMIDITY_H
#define TEMP_HUMIDITY_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "asf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "twi_interface.h"
#include "message_types.h"

typedef struct {
	QueueHandle_t Si7021_A20_temperature_data_queue;
	QueueHandle_t Si7021_A20_humidity_data_queue;

	int32_t temp;
	uint16_t rh;
	// twi_interface_t* twi_interface;

	int32_t Si7021_A20_tempurature;
	uint32_t Si7021_A20_humidity;

	//issue tasks hold
	twi_task_t issue_temp_read_hold;
	twi_task_t issue_rh_read_hold;

	//issue tasks no hold
	twi_task_t issue_temp_read_no_hold;
	twi_task_t issue_rh_read_no_hold;

	//receiving tasks
	twi_task_t receive_temp_read;
	twi_task_t receive_rh_read;
} Si7021_A20_t;

void Si7021_A20_init();

uint8_t Si7021_A20_issue_temp_read_holds();
uint8_t Si7021_A20_issue_rh_read_holds();

uint8_t Si7021_A20_issue_temp_read_noholds();
uint8_t Si7021_A20_issue_rh_read_noholds();


uint8_t Si7021_A20_receive_temp_read();
uint8_t Si7021_A20_receive_rh_read();

uint8_t Si7021_A20_fetch_queue_temp(sensors_message_t* curr_sensor_readings);
uint8_t Si7021_A20_fetch_queue_rh(sensors_message_t* curr_sensor_readings);

#endif

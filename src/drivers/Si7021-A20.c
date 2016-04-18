/*
Si7021-A20.c
Author: Matan Silver
Created: 2/2/2016
*/

#include "Si7021-A20.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "asf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#define SCLK_FREQ 400000
#define DEVICE_ADDRESS 0x40
#define MEASURE_RH_HOLD 0xE5
#define MEASURE_RH_NO_HOLD 0xF5
#define MEASURE_T_HOLD 0xE3
#define MEASURE_T_NO_HOLD 0xF3
#define READ_T_FROM_PREV_RH 0xE0
#define RESET 0xFE
#define WRITE_RH_T_REG1 0xE6
#define READ_RH_T_REG1 0xE7
#define WRITE_HEATER_CTRL_REG 0x51
#define READ_HEATER_CTRL_REG 0x11

#define DATA_QUEUE_DEPTH 5
//IO definitions
#define I2C_CS_PIN //????
//PE0 = SDA, PE1 = SCL

//Multiplicative offsets
#define TEMP_OFFSET 65536
#define TEMP_SCALAR 100
#define RH_OFFSET 65536
#define RH_SCALAR 125

Si7021_A20_t Si7021_A20;

static inline int32_t convert_to_celsius(uint16_t Temp_Code);

Si7021_A20_init()
{
	Si7021_A20.Si7021_A20_temperature_data_queue = xQueueCreate(DATA_QUEUE_DEPTH, sizeof(uint8_t));
	Si7021_A20.Si7021_A20_humidity_data_queue = xQueueCreate(DATA_QUEUE_DEPTH, sizeof(uint8_t));

    //issue temp read hold
    Si7021_A20.issue_temp_read_hold.device_addr = DEVICE_ADDRESS >> 1;
    Si7021_A20.issue_temp_read_hold.mode = TWI_WRITE_MODE;
    Si7021_A20.issue_temp_read_hold.write_data[0] = MEASURE_T_HOLD;
    Si7021_A20.issue_temp_read_hold.length = 1;

    //issue rh read hold
    Si7021_A20.issue_rh_read_hold.device_addr = DEVICE_ADDRESS >> 1;
    Si7021_A20.issue_rh_read_hold.mode = TWI_WRITE_MODE;
    Si7021_A20.issue_rh_read_hold.write_data[0] = MEASURE_RH_HOLD;
    Si7021_A20.issue_rh_read_hold.length = 1;

    //issue temp read no hold
    Si7021_A20.issue_temp_read_no_hold.device_addr = DEVICE_ADDRESS >> 1;
	Si7021_A20.issue_temp_read_no_hold.mode = TWI_WRITE_MODE;
	Si7021_A20.issue_temp_read_no_hold.write_data[0] = MEASURE_T_NO_HOLD;
	Si7021_A20.issue_temp_read_no_hold.length = 1;

    //issue rh read no hold
    Si7021_A20.issue_rh_read_no_hold.device_addr = DEVICE_ADDRESS >> 1;
	Si7021_A20.issue_rh_read_no_hold.mode = TWI_WRITE_MODE;
	Si7021_A20.issue_rh_read_no_hold.write_data[0] = MEASURE_RH_NO_HOLD;
	Si7021_A20.issue_rh_read_no_hold.length = 1;

    //get temp read
    Si7021_A20.receive_temp_read.return_queue = Si7021_A20.Si7021_A20_humidity_data_queue;
	Si7021_A20.receive_temp_read.device_addr = DEVICE_ADDRESS >> 1;
	Si7021_A20.receive_temp_read.mode = TWI_READ_MODE;
	Si7021_A20.receive_temp_read.length = 4;

    //get rh read
    Si7021_A20.receive_rh_read.return_queue = Si7021_A20.Si7021_A20_temperature_data_queue;
	Si7021_A20.receive_rh_read.device_addr = DEVICE_ADDRESS >> 1;
	Si7021_A20.receive_rh_read.mode = TWI_READ_MODE;
	Si7021_A20.receive_rh_read.length = 8;
}

uint8_t Si7021_A20_issue_temp_read_noholds() {
	twi_add_task_to_queue(&Si7021_A20.issue_temp_read_no_hold);
}
uint8_t Si7021_A20_issue_rh_read_noholds() {
	twi_add_task_to_queue(&Si7021_A20.issue_rh_read_no_hold);
}

uint8_t Si7021_A20_issue_temp_read_holds() {
	twi_add_task_to_queue(&Si7021_A20.issue_temp_read_hold);
}
uint8_t Si7021_A20_issue_rh_read_holds() {
	twi_add_task_to_queue(&Si7021_A20.issue_rh_read_hold);
}

uint8_t Si7021_A20_receive_temp_read() {
	twi_add_task_to_queue(&Si7021_A20.receive_temp_read);
}
uint8_t Si7021_A20_receive_rh_read() {
	twi_add_task_to_queue(&Si7021_A20.receive_rh_read);
}

static uint16_t convert_to_relative(uint16_t RH_Code) //takes RH_Code, returns percent relative humidity
{
	uint16_t RH = ((RH_SCALAR*RH_Code)-6*RH_OFFSET);
	if (RH < 0)
	{
		RH = 0;
	}
	else if (RH > 1)
	{
		RH = 100*RH_OFFSET;
	}
	return RH; //factor of RH_OFFSET larger than RH from 1 to 100
}
static inline int32_t convert_to_celsius(uint16_t Temp_Code)
{
	return ((17572*Temp_Code)-4685*TEMP_OFFSET); //offset by 6553600 (65536 * 100)
}

uint8_t Si7021_A20_fetch_queue_temp(sensors_message_t* curr_sensor_readings) {
	// uint32_t device_data = 0x0000; // Replaced with direct reference to D1
	uint8_t queue_result[2];
	uint8_t queue_index = 0;
	uint8_t status;

	status = 0;
	status = xQueuePeek(Si7021_A20.Si7021_A20_temperature_data_queue, &queue_result, 0);

	if (status == 1) {
		for (uint8_t queue_index = 0; queue_index < 3; queue_index++) {
			xQueueReceive(Si7021_A20.Si7021_A20_temperature_data_queue, &queue_result[queue_index], 0);
		}
		Si7021_A20.temp = convert_to_celsius(queue_result);
		curr_sensor_readings->Si7021_A20_temp = Si7021_A20.temp;
	}
	return status;
}

uint8_t Si7021_A20_fetch_queue_rh(sensors_message_t* curr_sensor_readings) {
	// uint32_t device_data = 0x0000; // Replaced with direct reference to D1
	uint8_t queue_result[2];
	uint8_t queue_index = 0;
	uint8_t status;

	status = 0;
	status = xQueuePeek(Si7021_A20.Si7021_A20_humidity_data_queue, &queue_result, 0);

	if (status == 1) {
		for (uint8_t queue_index = 0; queue_index < 3; queue_index++) {
			xQueueReceive(Si7021_A20.Si7021_A20_humidity_data_queue, &queue_result[queue_index], 0);
		}
		Si7021_A20.rh = convert_to_relative(queue_result);
		curr_sensor_readings->Si7021_A20_rh = Si7021_A20.rh;
	}
	return status;
}

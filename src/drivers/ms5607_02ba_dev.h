/*
 * ms5607_02ba_dev.h
 *
 * Created: 4/3/2016 6:21:15 PM
 *  Author: Tim Rupprecht
 */ 

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "asf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "twi_interface.h"

#ifndef MS5607_02BA_DEV_H_
#define MS5607_02BA_DEV_H_

/* Public Types */
typedef int32_t rocket_temp_t;
typedef int32_t rocket_press_t;

enum ms5607_02ba_osr {
	OSR_256 = 0x0,
	OSR_512 = 0x1,
	OSR_1024 = 0x2,
	OSR_2048 = 0x3,
	OSR_4096 = 0x4
};

/* Private Types */
typedef struct {
	uint16_t manufacturer;
	uint16_t coefficient_1;
	uint16_t coefficient_2;
	uint16_t coefficient_3;
	uint16_t coefficient_4;
	uint16_t coefficient_5;
	uint16_t coefficient_6;
	uint16_t crc;
} prom_t;

typedef struct ms5607_02ba_dev {
	QueueHandle_t ms5607_02ba_data_queue;
	twi_interface_t* twi_interface;
	
	rocket_temp_t ms5607_02ba_tempurature;
	rocket_press_t ms5607_02ba_pressure;
	
	prom_t prom_data;
	
	
	twi_task_t resetDevice;
	
	twi_task_t preparePromReg;
	twi_task_t getPromReg;
	
	twi_task_t prepareD1;
	twi_task_t prepareD2;
	
	twi_task_t prepareADC;
	twi_task_t getADC;
	
} ms5607_02ba_dev_t;

// initialization functions
void ms5607_02ba_init(twi_interface_t*);
uint8_t ms5607_02ba_reset(void);
uint8_t ms5607_02ba_load_prom(void);

// Commands / Reading Data
uint8_t ms5607_02ba_convert_d1(void);
uint8_t ms5607_02ba_convert_d2(void);
uint8_t ms5607_02ba_read_adc(uint32_t* adc_value);

uint8_t ms5607_02ba_fetch_queue_data(void);

// Clean up raw data
rocket_temp_t ms5607_02ba_calculate_temp(uint32_t d2);
rocket_press_t ms5607_02ba_calculate_press(uint32_t d1, uint32_t d2);

// Processing data
uint8_t ms5607_02ba_fetch_queue_data(void);

#endif /* MS5607_02BA_DEV_H_ */
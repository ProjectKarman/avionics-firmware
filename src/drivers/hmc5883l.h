/*
 * hmc5883l.h
 *
 * Created: 4/7/2016 8:51:32 PM
 *  Author: Andrew Kaster
 */ 


#ifndef HMC5883L_H_
#define HMC5883L_H_
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "twi_interface.h"

//Min and max values stored in data registers
#define HMC5883L_OUTPUT_MIN		 0xF800
#define HMC5883L_OUTPUT_MAX		 0x07FF

//In the event of ADC over/underflow or math overflow during bias calc
//Data register will have the value of -4096 until next valid measurement
#define HMC5883L_OUTPUT_ERROR	 0xF000



//Data is base 16
typedef struct hmc5883l_rawdata{
  uint16_t x_magnitude;
  uint16_t y_magnitude;
  uint16_t z_magnitude;
} hmc5883l_rawdata_t;

typedef struct hmc5883l {
  QueueHandle_t hmc5883l_data_queue;
  
  hmc5883l_rawdata_t magnetometer_data;

  twi_task_t setConfigA;
  twi_task_t setConfigB;
  twi_task_t setMode;
  twi_task_t getRawData;
  twi_task_t resetRegPointer;


} hmc5883l_t;


/* Initialize TWI (I2C) interface */
void hmc5883l_init();

/* Configure registers for power-on initialization */
uint8_t hmc5883l_configure();

/*Read raw data from magnetometer
	Sends I2C messages to module to read starting at register 3 (XMSB)
	Stores the result in a rawdata_t struct passed as a parameter
*/
void hmc5883l_read_data_single (hmc5883l_rawdata_t* raw_data_ptr);

/*Read raw data from magnetometer
	Sends I2C messages to module to read starting at register 3 (XMSB)
	Stores the result in a rawdata_t struct passed as a parameter
*/
uint8_t hmc5883l_read_data_continuous (hmc5883l_rawdata_t *rawdata_ptr);


#endif /* HMC5883L_H_ */
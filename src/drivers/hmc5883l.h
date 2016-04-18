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
#include "message_types.h"

//Data is base 16
typedef struct hmc5883l_rawdata{
  int16_t x_magnitude;
  int16_t y_magnitude;
  int16_t z_magnitude;
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
	Sends TWI messages to module to read starting at register 3 (XMSB)
	Stores the result in a rawdata_t struct passed as a parameter
*/
//void hmc5883l_read_data_single (hmc5883l_rawdata_t* raw_data_ptr); // Do we need this?

/*Read raw data from magnetometer
  Sends TWI message to module to read starting at register 3 (XMSB)
  */
  uint8_t hmc5883l_read_data();


/*Read raw data from queue
	Stores the result in a rawdata_t struct passed as a parameter
*/
uint8_t hmc5883l_fetch_queue_data (sensors_message_t *curr_sensor_readings);


//Should we do anything with this raw data? Or just pass it straight up without modification?


#endif /* HMC5883L_H_ */
/*
 * hmc5883l.h
 *
 * Created: 2/11/2016 9:35:12 PM
 *  Author: Andrew Kaster
 */ 


#ifndef HMC5883L_H_
#define HMC5883L_H_
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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

//Angles are base 10
typedef struct {
	int yaw;
	int pitch;
	int roll;
} hmc5883l_angle_t;

//Function callback type
typedef void (*hmc5883l_callback_t)(void);

/* Initialize TWI (I2C) interface and prepare OS level structures */
void hmc5883l_init(void);

/*Read raw data from magnetometer
	Sends I2C messages to module to read starting at register 3 (XMSB)
	Stores the result in a rawdata_t struct passed as a parameter
*/
void hmc5883l_read_data_single (hmc5883l_rawdata_t* raw_data_ptr);

/*Read raw data from magnetometer
	Sends I2C messages to module to read starting at register 3 (XMSB)
	Stores the result in a rawdata_t struct passed as a parameter
*/
void hmc5883l_read_data_continuous (hmc5883l_rawdata_t *rawdata_ptr);

/*Convert raw data to yaw pitch roll
	takes copy of raw data and returns an angle_t struct
*/
void hmc5883l_get_angles(hmc5883l_rawdata_t rawdata);

#endif /* HMC5883L_H_ */
/*
 * hmc5883l.h
 *
 * Created: 2/11/2016 9:35:12 PM
 *  Author: Andrew Kaster
 */ 


#ifndef HMC5883L_H_
#define HMC5883L_H_

//Min and max values stored in data registers
#define HMC5883L_OUTPUT_MIN		 0xF800
#define HMC5883L_OUTPUT_MAX		 0x07FF

//In the event of ADC over/underflow or math overflow during bias calc
//Data register will have the value of -4096 until next valid measurement
#define HMC5883L_OUTPUT_ERROR	 0xF000 


enum hmc5883l_sensor_mode_t{
	HMC5883L_MODE_CONTINUOUS,
	HMC5883L_MODE_SINGLE,
	HMC5883L_MODE_IDLE
};

enum hmc5883l_reg_t {
	HMC5883L_CRA,		//Control register A
	HMC5883L_CRB,		//Control register B
	HMC5883L_MODE,		
	HMC5883L_XMSB,
	HMC5883L_XLSB,
	HMC5883L_ZMSB,
	HMC5883L_ZLSB,
	HMC5883L_YMSB,
	HMC5883L_YLSB,
	HMC5883L_STATUS,
	HMC5883L_IDA,		//Identification register A
	HMC5883L_IDB,		//Identification register B
	HMC5883L_IDC		//Identification register C
};


//Data is base 16
typedef struct {
	int16_t x_magnitude;
	int16_t y_magnitude;
	int16_t z_magnitude;
} hmc5883l_rawdata_t;

//Angles are base 10
typedef struct {
	int yaw;
	int pitch;
	int roll;
} hmc5883l_angle_t;

/*Read raw data from magnetometer
	PRECONDITION: measurement mode has been set to single
	Sends I2C messages to module to read starting at register 3 (XMSB)
	Stores the result in a rawdata_t struct passed as a parameter
*/
void hmc5883l_read_data_single (hmc5883l_rawdata_t *rawdata_ptr);

/*Read raw data from magnetometer
	PRECONDITION: measurement mode has been set to continuous
	Sends I2C messages to module to read starting at register 3 (XMSB)
	Stores the result in a rawdata_t struct passed as a parameter
*/
void hmc5883l_read_data_continuous (hmc5883l_rawdata_t *rawdata_ptr);

/*Convert raw data to yaw pitch roll
	takes copy of raw data and returns an angle_t struct
*/
hmc5883l_angle_t hmc5883l_get_angles(hmc5883l_rawdata_t rawdata);

#endif /* HMC5883L_H_ */
/*
 * hmc5883l.h
 *
 * Created: 2/11/2016 9:35:12 PM
 *  Author: Andrew Kaster
 */ 


#ifndef HMC5883L_H_
#define HMC5883L_H_

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



#endif /* HMC5883L_H_ */
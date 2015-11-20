/*
 * ms5607_02ba.h
 *
 * Created: 10/03/2015 11:55:36 PM
 *  Author: Timothy Rupprecht
 */ 

#include <stdint.h>

#ifndef MS5607_02BA_H_
#define MS5607_02BA_H_

// tmep/press for the rocket
typedef int32_t rocket_temp_t;
typedef int32_t rocket_press_t;

// Resolution
enum ms5607_02ba_osr {
  OSR_256,
  OSR_512, 
  OSR_1024, 
  OSR_2048, 
  OSR_4096
};

void ms5607_02ba_init(void);
void ms5607_02ba_reset(void);
void ms5607_02ba_convert_D1(enum ms5607_02ba_osr osr);
void ms5607_02ba_convert_D2(enum ms5607_02ba_osr osr);
void ms5607_02ba_read_adc(uint32_t* adc_value);
void ms5607_02ba_read_prom(void);
rocket_temp_t ms5607_02ba_calculate_temp(uint32_t d2);
rocket_press_t ms5607_02ba_calculate_press(uint32_t d1, uint32_t d2);

#endif /* MS5607_02BA_H_ */
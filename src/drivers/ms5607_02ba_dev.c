/*
 * ms5607_02ba_dev.c
 *
 * Created: 4/3/2016 6:58:58 PM
 *  Author: Tim Rupprecht
 */ 


#include "ms5607_02ba_dev.h"

// Device Params
#define DEVICE_ADDRESS 0x77
#define DEVICE_RESET_DELAY 3
#define DEVICE_MAX_DATA_LEN 3

// Device Command Set
#define CMD_RESET             0x1E
#define CMD_CONVERTD1(osr)   (0x40 | (osr << 1))
#define CMD_CONVERTD2(osr)   (0x50 | (osr << 1))
#define CMD_ADC_READ          0x00
#define CMD_READ_REG(reg)    (0xA0 | (reg << 1))

// Offsets for raw data conversions to temp/pressure (SEE DATASHEET)
#define DT_MULTIPLICATIVE_OFFSET 256 // 2^8
#define T_MULTIPLICATIVE_OFFSET 8388608 // 2^23
#define OFF_MULTIPLICATIVE_OFFSET_1 131072 // 2^17
#define OFF_MULTIPLICATIVE_OFFSET_2 64 // 2^6
#define SENS_MULTIPLICATIVE_OFFSET_1 65536 // 2^16
#define SENS_MULTIPLICATIVE_OFFSET_2 128 // 2^7
#define P_MULTIPLICATIVE_OFFSET_1 2097152 // 2^21
#define P_MULTIPLICATIVE_OFFSET_2 32768 // 2^1

#define MAG_SHIFT 1024 // 2^10

#define DATA_QUEUE_DEPTH 6

twi_task_t currentTask;
ms5607_02ba_dev_t ms5607_02ba;

// initialization functions
void ms5607_02ba_init(twi_interface_t* twi_obj) {
	ms5607_02ba.ms5607_02ba_data_queue = xQueueCreate(DATA_QUEUE_DEPTH, sizeof(uint8_t));
	ms5607_02ba.twi_interface = twi_obj;
	
	for (uint8_t prom_addr = 0; prom_addr < 8; prom_addr++) {
		*((uint16_t *)&ms5607_02ba.prom_data + prom_addr) = 0x00;
	}
	
	// This does not need to change in ms5607_02ba_reset()
	ms5607_02ba.resetDevice.return_queue = ms5607_02ba.ms5607_02ba_data_queue;
	ms5607_02ba.resetDevice.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.resetDevice.mode = TWI_WRITE_MODE;
	ms5607_02ba.resetDevice.write_data = CMD_RESET;
	ms5607_02ba.resetDevice.length = 1;
	
	// This will need to change in ms5607_02ba_load_prom() to be specific to ms5607_02ba's
	// specific register you are hoping to read from
	ms5607_02ba.preparePromReg.return_queue = ms5607_02ba.ms5607_02ba_data_queue;
	ms5607_02ba.preparePromReg.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.preparePromReg.mode = TWI_WRITE_MODE;
	ms5607_02ba.preparePromReg.write_data = 0x00;
	ms5607_02ba.preparePromReg.length = 1;
	
	// This does not need to change in ms5607_02ba_load_prom() because it is not
	// specific to the register you are hoping to read from
	ms5607_02ba.getPromReg.return_queue = ms5607_02ba.ms5607_02ba_data_queue;
	ms5607_02ba.getPromReg.device_addr = DEVICE_ADDRESS << 1 | 0x1;
	ms5607_02ba.getPromReg.mode = TWI_READ_MODE;
	ms5607_02ba.getPromReg.write_data = 0x00;
	ms5607_02ba.getPromReg.length = 2;
	
	// This will need to change in ms5607_02ba_convert_d*() to be specific to ms5607_02ba's
	// specific data you are hoping to receive from the device
	ms5607_02ba.prepareD1.return_queue = ms5607_02ba.ms5607_02ba_data_queue;
	ms5607_02ba.prepareD1.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.prepareD1.mode = TWI_WRITE_MODE;
	ms5607_02ba.prepareD1.write_data = CMD_CONVERTD1(OSR_4096);
	ms5607_02ba.prepareD1.length = 1;
	
	// This will need to change in ms5607_02ba_convert_d*() to be specific to ms5607_02ba's
	// specific data you are hoping to receive from the device
	ms5607_02ba.prepareD2.return_queue = ms5607_02ba.ms5607_02ba_data_queue;
	ms5607_02ba.prepareD2.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.prepareD2.mode = TWI_WRITE_MODE;
	ms5607_02ba.prepareD2.write_data = CMD_CONVERTD2(OSR_4096);;
	ms5607_02ba.prepareD2.length = 1;
		
	// This won't need to change in ms5607_02ba_convert_d*() because it isn't specific to
	// ms5607_02ba's specific data you are hoping to receive from the device
	ms5607_02ba.prepareADC.return_queue = ms5607_02ba.ms5607_02ba_data_queue;
	ms5607_02ba.prepareADC.device_addr = DEVICE_ADDRESS << 1 | 0x1;
	ms5607_02ba.prepareADC.mode = TWI_WRITE_MODE;
	ms5607_02ba.prepareADC.write_data = CMD_ADC_READ;
	ms5607_02ba.prepareADC.length = 1;
	
	// This won't need to change in ms5607_02ba_convert_d*() because it isn't specific to
	// ms5607_02ba's specific data you are hoping to receive from the device
	ms5607_02ba.getADC.return_queue = ms5607_02ba.ms5607_02ba_data_queue;
	ms5607_02ba.getADC.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.getADC.mode = TWI_READ_MODE;
	ms5607_02ba.getADC.write_data = 0x00;
	ms5607_02ba.getADC.length = 3;
}

uint8_t ms5607_02ba_reset(void) {
	twi_add_task_to_queue(&ms5607_02ba.resetDevice);
}

uint8_t ms5607_02ba_load_prom(void) {
	for (uint8_t prom_addr = 0; prom_addr < 8; prom_addr++)  {
		ms5607_02ba.preparePromReg.write_data = CMD_READ_REG(prom_addr);
		
		twi_add_task_to_queue(&ms5607_02ba.preparePromReg);
		twi_add_task_to_queue(&ms5607_02ba.getPromReg);
		
		// TODO Process receive queue (blocking)
	}
}

// Commands / Reading Data
uint8_t ms5607_02ba_convert_d1() {
	twi_add_task_to_queue(&ms5607_02ba.prepareD1);
}

uint8_t ms5607_02ba_convert_d2() {
	twi_add_task_to_queue(&ms5607_02ba.prepareD2);
}

uint8_t ms5607_02ba_prepare_adc(uint32_t* adc_value) {
	twi_add_task_to_queue(&ms5607_02ba.prepareADC);
	twi_add_task_to_queue(&ms5607_02ba.prepareADC);
	
	// PROCESS QUEUE NON_BLOCKING - RETURNS STATUS
}

// Clean up raw data
rocket_temp_t ms5607_02ba_calculate_temp(uint32_t d2) {
	
}

rocket_press_t ms5607_02ba_calculate_press(uint32_t d1, uint32_t d2) {
	
}
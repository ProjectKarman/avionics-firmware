/*
 * ms5607_02ba_dev.c
 *
 * Created: 4/3/2016 6:58:58 PM
 *  Author: Tim Rupprecht
 */ 


#include "ms5607_02ba_dev.h"
#include "message_types.h"

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
prom_t prom_data;

// initialization functions
// NOTE: Does the altimeter need reference to the twi object? I don;t think so. Further testing needed.
void ms5607_02ba_init(/*twi_interface_t* twi_obj*/) {
	ms5607_02ba.ms5607_02ba_prom_reg_data_queue = xQueueCreate(DATA_QUEUE_DEPTH, sizeof(uint8_t));
	ms5607_02ba.ms5607_02ba_temperature_data_queue = xQueueCreate(DATA_QUEUE_DEPTH, sizeof(uint8_t));
	ms5607_02ba.ms5607_02ba_pressure_data_queue = xQueueCreate(DATA_QUEUE_DEPTH, sizeof(uint8_t));
	
	// ms5607_02ba.twi_interface = twi_obj;
	
	for (uint8_t prom_addr = 0; prom_addr < 8; prom_addr++) {
		*((uint16_t *)&ms5607_02ba.prom_data + prom_addr) = 0x00;
	}
	
	// This does not need to change in ms5607_02ba_reset()
	// NO RETURN QUEUE UNLESS READ INSTRUCTION
	ms5607_02ba.resetDevice.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.resetDevice.mode = TWI_WRITE_MODE;
	ms5607_02ba.resetDevice.write_data[0] = CMD_RESET;
	ms5607_02ba.resetDevice.length = 1;
	
	// This will need to change in ms5607_02ba_load_prom() to be specific to ms5607_02ba's
	// specific register you are hoping to read from
	ms5607_02ba.preparePromReg.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.preparePromReg.mode = TWI_WRITE_MODE;
	ms5607_02ba.preparePromReg.write_data[0] = 0x00;
	ms5607_02ba.preparePromReg.length = 1;
	
	// This does not need to change in ms5607_02ba_load_prom() because it is not
	// specific to the register you are hoping to read from
	ms5607_02ba.getPromReg.return_queue = ms5607_02ba.ms5607_02ba_prom_reg_data_queue;
	ms5607_02ba.getPromReg.device_addr = DEVICE_ADDRESS << 1 | 0x1;
	ms5607_02ba.getPromReg.mode = TWI_READ_MODE;
	ms5607_02ba.getPromReg.length = 2;
	
	// This will need to change in ms5607_02ba_convert_d*() to be specific to ms5607_02ba's
	// specific data you are hoping to receive from the device
	ms5607_02ba.prepareD1.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.prepareD1.mode = TWI_WRITE_MODE;
	ms5607_02ba.prepareD1.write_data[0] = CMD_CONVERTD1(OSR_4096);
	ms5607_02ba.prepareD1.length = 1;
	
	// This will need to change in ms5607_02ba_convert_d*() to be specific to ms5607_02ba's
	// specific data you are hoping to receive from the device
	ms5607_02ba.prepareD2.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.prepareD2.mode = TWI_WRITE_MODE;
	ms5607_02ba.prepareD2.write_data[0] = CMD_CONVERTD2(OSR_4096);;
	ms5607_02ba.prepareD2.length = 1;
		
	// This won't need to change in ms5607_02ba_convert_d*() because it isn't specific to
	// ms5607_02ba's specific data you are hoping to receive from the device
	ms5607_02ba.prepareADC.device_addr = DEVICE_ADDRESS << 1 | 0x1;
	ms5607_02ba.prepareADC.mode = TWI_WRITE_MODE;
	ms5607_02ba.prepareADC.write_data[0] = CMD_ADC_READ;
	ms5607_02ba.prepareADC.length = 1;
	
	// This won't need to change in ms5607_02ba_convert_d*() because it isn't specific to
	// ms5607_02ba's specific data you are hoping to receive from the device
	ms5607_02ba.getADC_temperature.return_queue = ms5607_02ba.ms5607_02ba_temperature_data_queue;
	ms5607_02ba.getADC_temperature.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.getADC_temperature.mode = TWI_READ_MODE;
	ms5607_02ba.getADC_temperature.length = 3;
	
	// This won't need to change in ms5607_02ba_convert_d*() because it isn't specific to
	// ms5607_02ba's specific data you are hoping to receive from the device
	ms5607_02ba.getADC_pressure.return_queue = ms5607_02ba.ms5607_02ba_pressure_data_queue;
	ms5607_02ba.getADC_pressure.device_addr = DEVICE_ADDRESS << 1;
	ms5607_02ba.getADC_pressure.mode = TWI_READ_MODE;
	ms5607_02ba.getADC_pressure.length = 3;
}

// Must reset the device before use at start up
uint8_t ms5607_02ba_reset(void) {
	twi_add_task_to_queue(&ms5607_02ba.resetDevice);
}

// You need to load the altimeter's device constants off of it
// to be used in the conversion from the device's raw data to 
// a readable format
uint8_t ms5607_02ba_load_prom(void) {
	for (uint8_t prom_addr = 0; prom_addr < 8; prom_addr++)  {
		// CMD_READ_REG(prom_addr) = (0xA0 | (prom_addr << 1))
		ms5607_02ba.preparePromReg.write_data[0] = CMD_READ_REG(prom_addr);
		
		twi_add_task_to_queue(&ms5607_02ba.preparePromReg);
		twi_add_task_to_queue(&ms5607_02ba.getPromReg);
		twi_process_queue_blocking();
	}
	for (uint8_t prom_addr = 0; prom_addr < 8; prom_addr++) {
		*((uint16_t *)&prom_data + prom_addr) = ms5607_02ba_fetch_queue_data_blocking();
	}
}

// Commands / Reading Data
// Prepare D1 for adc read
uint8_t ms5607_02ba_convert_d1() {
	twi_add_task_to_queue(&ms5607_02ba.prepareD1);
}

// Prepare D2 for adc read
uint8_t ms5607_02ba_convert_d2() {
	twi_add_task_to_queue(&ms5607_02ba.prepareD2);
}

// Prepare to read the analog to digital converter with both write and
// resulting read commands. They're sent after the time required to prepare
// their previous convert d1 or 2 command.
uint8_t ms5607_02ba_prepare_adc(MS5607_02BA_POLL_MODE_T polling_mode) {
	twi_add_task_to_queue(&ms5607_02ba.prepareADC);
	
	if (polling_mode == START_UP || polling_mode == TEMPURATURE) {
		twi_add_task_to_queue(&ms5607_02ba.getADC_temperature);
	}
	else {
		twi_add_task_to_queue(&ms5607_02ba.getADC_pressure);
	}
}

// Clean up raw data
rocket_temp_t ms5607_02ba_calculate_temp(uint32_t d2) {
    rocket_temp_t t;
    rocket_temp_t dt;
    rocket_temp_t tempvar1;
		  
    dt = d2 - ((int32_t) prom_data.coefficient_5 * DT_MULTIPLICATIVE_OFFSET);
	tempvar1 = (dt * ( prom_data.coefficient_6 / T_MULTIPLICATIVE_OFFSET));
	t = ( (int32_t) 2000 * MAG_SHIFT) + tempvar1;
	t = (int32_t) t / MAG_SHIFT;
	return t;
}

rocket_press_t ms5607_02ba_calculate_press(uint32_t d1, uint32_t d2) {
	  rocket_press_t p;
	  int32_t dt;
	  int64_t off;
	  int64_t sens;
	  int64_t tempvar1;
	  int64_t tempvar2;
	  
	  dt = d2 - ((int32_t) prom_data.coefficient_5 * DT_MULTIPLICATIVE_OFFSET);
	  
	  tempvar2 = (int64_t) prom_data.coefficient_4 * dt;
	  tempvar1 = (int64_t) prom_data.coefficient_2 * OFF_MULTIPLICATIVE_OFFSET_1;
	  off = tempvar1 + ((int64_t) tempvar2 / OFF_MULTIPLICATIVE_OFFSET_2);
	  
	  tempvar1 = (int64_t) prom_data.coefficient_1 * SENS_MULTIPLICATIVE_OFFSET_1;
	  tempvar2 =  ( (int64_t) ( (int64_t) prom_data.coefficient_3 * dt) / SENS_MULTIPLICATIVE_OFFSET_2);
	  sens = tempvar1 + tempvar2;
	  
	  
	  tempvar1 = (int64_t) d1 * sens;
	  tempvar2 =  (int64_t) tempvar1 / P_MULTIPLICATIVE_OFFSET_1;
	  p = (int64_t) (tempvar2 - off) / P_MULTIPLICATIVE_OFFSET_2;
}

// Process Data
uint8_t ms5607_02ba_fetch_queue_temp(sensors_message_t* curr_sensor_readings) {
	// uint32_t device_data = 0x0000; // Replaced with direct reference to D1
	uint8_t queue_result[2];
	uint8_t queue_index = 0;
	rocket_temp_t rocket_temp;
	uint8_t status;
	
	status = 0;
	status = xQueuePeek(ms5607_02ba.ms5607_02ba_temperature_data_queue, &queue_result, 0);
	
	if (status == 1) {
		for (uint8_t queue_index = 0; queue_index < 3; queue_index++) {
			xQueueReceive(ms5607_02ba.ms5607_02ba_temperature_data_queue, &queue_result[queue_index], 0);
		}
		ms5607_02ba.d1 = convert_buffer_24(queue_result);
		rocket_temp = ms5607_02ba_calculate_temp(ms5607_02ba.d1);
		curr_sensor_readings->rocket_tempurature = rocket_temp;
	}
	
	return status;
}

uint8_t ms5607_02ba_fetch_queue_press(sensors_message_t* curr_sensor_readings) {
	// uint32_t device_data = 0x0000; //Replaced with direct reference to D2
	uint8_t queue_result[2];
	rocket_press_t rocket_pressure;
	uint8_t status;
	
	status = 0;
	status = xQueuePeek(ms5607_02ba.ms5607_02ba_pressure_data_queue, &queue_result, 0);
	
	if (status == 1) {
		for (uint8_t queue_index = 0; queue_index < 3; queue_index++) {
			xQueueReceive(ms5607_02ba.ms5607_02ba_pressure_data_queue, &queue_result[queue_index], 0);
		}
		ms5607_02ba.d2 = convert_buffer_24(queue_result);
		rocket_pressure = ms5607_02ba_calculate_press(ms5607_02ba.d1, ms5607_02ba.d2);
		curr_sensor_readings->rocket_pressure = rocket_pressure;
	}
	
	return status;
}

uint16_t ms5607_02ba_fetch_queue_data_blocking() {
	uint16_t prom_data = 0x00;
	uint8_t queue_result[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
	uint8_t status;
	
	status = xQueuePeek(ms5607_02ba.ms5607_02ba_prom_reg_data_queue, &queue_result, portMAX_DELAY);

	if (status == 1) {
		for (uint8_t queue_index = 0; queue_index < 6; queue_index++) {
			xQueueReceive(ms5607_02ba.ms5607_02ba_prom_reg_data_queue, &queue_result[queue_index], portMAX_DELAY);
		}
		prom_data = convert_buffer_16(queue_result);
	}
	
	return prom_data;
}

static inline uint32_t convert_buffer_24(uint8_t *buffer) {
	return ((uint32_t)buffer[0] << 8*2) | ((uint32_t)buffer[1] << 8*1) | ((uint32_t)buffer[2] << 8*0);
}

static inline uint16_t convert_buffer_16(uint8_t *buffer) {
	return ((uint16_t)buffer[0] << 8*1) | ((uint16_t)buffer[1] << 8*0);
}
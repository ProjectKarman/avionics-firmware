/*
Si7021-A20.c
Author: Matan Silver
Created: 2/2/2016
*/

#include <Si7021-A20.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "asf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#define SCLK_FREQ 400000
#define SEVEN_BIT_SLAVE_ADDRESS 0x40
#define MEASURE_RH_HOLD 0xE5
#define MEASURE_RH_NO_HOLD 0xF5
#define MEASURE_T_HOLD 0xE3
#define MEASURE_T_NO_HOLD 0xF3
#define READ_T_FROM_PREV_RH 0xE0
#define RESET 0xFE
#define WRITE_RH_T_REG1 0xE6
#define READ_RH_T_REG1 0xE7
#define WRITE_HEATER 0x51
#define READ_HEATER 0x11
#define READ_1ST_BYTE 0xFA
#define READ_1ST_BYTE_2 0x0F
#define READ_2ND_BYTE 0xFC
#define READ_2ND_BYTE_2 0xC9
#define READ_FW_REV 0x84
#define READ_FW_REV 0xB8

//IO definitions
#define I2C_CS_PIN //????
//PE0 = SDA, PE1 = SCL

//Multiplicative offsets
#define TEMP_OFFSET 65536
#define TEMP_SCALAR 100
#define RH_OFFSET 65536
#define RH_SCALAR 125

static void i2c_start(void);
static void i2c_end(void);
static void send_command(uint8_t cmd);
static void send_command_async(uint8_t cmd, si7021_callback_t callback);

void Si7021_A20_measure_temp()
{

}
void Si7021_A20_measure_rh()
{

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
static uint32_t convert_to_celsius(uint16_t Temp_Code)
{
	return ((17572*Temp_Code)-4685*TEMP_OFFSET); //offset by 6553600 (65536 * 100)
}
static uint32_t retrieve_serial()
{
	//no params
	//returns serial number of chip, comes in 8 data bytes
	//uses two I2C commands
	i2c_start(); //first access
	i2c_end();

	i2c_start(); //second access
	i2c_end();
}
static uint64_t retrieve_firmware_rev()
{
	uint64_t fw_rev;
	//no params
	//returns firmware revision of chip
	i2c_start();
	//slave address
	//write
	//acknowledge
	//0x84
	//acknowledge
	//0xB8
	//acknowledge
	i2c_start(); //why twice
	//slave address
	//read
	//acknowledge
	//firmware revision comes in
	//acknowledge
	//NA????
	i2c_end();
	return fw_rev; //0xFF = firmware rev 1.0, 0x20 = firmware rev 2.0
}
static void i2c_start(void)
{

}
static void i2c_end(void)
{

}
static void send_command(uint8_t cmd)
{
  *op_buffer = cmd;
  op_buffer_index = 0;
  op_buffer_len = 1;
  current_command_type = CMD_TYPE_WRITE;
  TWI_MASTER.MASTER.ADDR = DEVICE_ADDRESS << 1;
  //xSemaphoreTake(command_complete_semaphore, portMAX_DELAY); // Wait for command to complete
}

static void send_command_async(uint8_t cmd, si7021_callback_t callback) {
  *op_buffer = cmd;
  op_buffer_index = 0;
  op_buffer_len = 1;
  current_command_type = CMD_TYPE_WRITE_ASYNC;
  current_op_callback = callback;
  TWI_MASTER.MASTER.ADDR = DEVICE_ADDRESS << 1;
}

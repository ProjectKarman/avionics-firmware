#include "MS5607_02BA.h"
#include "asf.h"

// I2C Command Set
#define CMD_RESET             0x1E
#define CMD_CONVERTD1_256     0x40
#define CMD_CONVERTD1_512     0x42
#define CMD_CONVERTD1_1024    0x44
#define CMD_CONVERTD1_2048    0x46
#define CMD_CONVERTD1_4096    0x48
#define CMD_CONVERTD2_256     0x50
#define CMD_CONVERTD2_512     0x52
#define CMD_CONVERTD2_1024    0x54
#define CMD_CONVERTD2_2048    0x56
#define CMD_CONVERTD2_4096    0x58
#define CMD_ADC_READ          0x00
#define CMD_READ_REG(reg)  (0xA0 | (reg << 1)  )

#define DEVICE_ADDRESS 0x77

// TWI Peripheral Config
#define TWI_MASTER       TWIE
#define TWI_MASTER_PORT  PORTE
#define TWI_FREQ        400000
#define TWI_BAUD_REG 35 // ((F_SYS / (2 * TWI_FREQ)) - 5)

#define DT_MULTIPLICATIVE_OFFSET 256 // 2^8
#define T_MULTIPLICATIVE_OFFSET 8388608 // 2^23
#define OFF_MULTIPLICATIVE_OFFSET_1 131072 // 2^17
#define OFF_MULTIPLICATIVE_OFFSET_2 64 // 2^6
#define SENS_MULTIPLICATIVE_OFFSET_1 65536 // 2^16
#define SENS_MULTIPLICATIVE_OFFSET_2 128 // 2^7
#define P_MULTIPLICATIVE_OFFSET_1 2097152 // 2^21
#define P_MULTIPLICATIVE_OFFSET_2 32768 // 2^15

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

static prom_t prom_data;

static void send_command(uint8_t cmd);
static void get_data(uint8_t *buffer, uint8_t buffer_len);

void ms5607_02ba_init(void) {
  TWI_MASTER_PORT.PIN0CTRL |= PORT_OPC_WIREDANDPULL_gc;
  TWI_MASTER_PORT.PIN1CTRL |= PORT_OPC_WIREDANDPULL_gc;
  
  
  TWI_MASTER.MASTER.BAUD = 35;
  TWI_MASTER.MASTER.CTRLA |= TWI_MASTER_ENABLE_bm;
}

void ms5607_02ba_reset(void) {
  send_command(CMD_RESET);
}

/*
void ms5607_02ba_convert_D1(enum ms5607_02ba_osr osr) {
  switch(osr) {
    case OSR_4096
    send_command(CMD_CONVERTD1_4096);
    break;
    case OSR_2048
    send_command(CMD_CONVERTD1_2048);
    break;
    case OSR_1024
    send_command(CMD_CONVERTD1_1024);
    break;
    case OSR_512
    send_command(CMD_CONVERTD1_512);
    break;
    case OSR_256
    send_command(CMD_CONVERTD1_256);
    break;
  }
}
void ms5607_02ba_convert_D2(enum ms5607_02ba_osr osr) {
  switch(osr) {
    case OSR_4096
      send_command(CMD_CONVERTD2_4096);
      break;
    case OSR_2048
      send_command(CMD_CONVERTD2_2048);
      break;
    case OSR_1024
      send_command(CMD_CONVERTD2_1024);
      break;
    case OSR_512
      send_command(CMD_CONVERTD2_512);
      break;
    case OSR_256
      send_command(CMD_CONVERTD2_256);
      break;
  }
}
*/

void ms5607_02ba_read_adc(uint32_t *adc_value) {
  uint8_t buffer[3];
  send_command(CMD_ADC_READ);
  get_data(buffer, 3);
  *adc_value = ((uintptr_t)buffer[2] << 8*2) | ((uintptr_t)buffer[1] << 8*1) | ((uintptr_t)buffer[0] << 8*0);
}

void ms5607_02ba_read_prom(void) { 
  uint8_t data_buffer[2];
  
  for (uint8_t prom_addr = 0; prom_addr < 8; prom_addr++) {
    send_command(CMD_READ_REG(prom_addr));
    get_data(data_buffer, 2);
    
    *((uint16_t *)&prom_data + prom_addr) = data_buffer[0] << 8 | data_buffer[1];
  }
}

static void send_command(uint8_t cmd)
{
  TWI_MASTER.MASTER.ADDR = DEVICE_ADDRESS;
  
  /*
  twi_package_t packet = {
    .addr         = {cmd}, // TWI slave memory address data
    .addr_length  = 1,  // TWI slave memory address data size
    .chip         = CHIP_ADDRESS, // TWI slave bus address
    .buffer       = NULL,  // transfer data destination buffer
    .length       = 0, // transfer buffer data size (bytes)
    .no_wait      = false
  };
  
  twi_master_write(&TWI_MASTER, &packet);
  */
}

static void get_data(uint8_t *buffer, uint8_t buffer_len) {
  /*
  twi_package_t packet = {
    .addr         = NULL, // TWI slave memory address data
    .addr_length  = 0,  // TWI slave memory address data size
    .chip         = DEVICE_ADDRESS, // TWI slave bus address
    .buffer       = buffer,  // transfer data destination buffer
    .length       = buffer_len, // transfer buffer data size (bytes)
    .no_wait      = false
  };
  
  twi_master_read(&TWI_MASTER, &packet);
  */
}

rocket_temp_t ms5607_02ba_calculate_temp(uint32_t d2) {
  rocket_temp_t t;
  int32_t dt;

  dt = d2 - ( prom_data.coefficient_5 * DT_MULTIPLICATIVE_OFFSET );
  t = 2000 + (dt * ( prom_data.coefficient_6 / T_MULTIPLICATIVE_OFFSET ));
  return t;
}

rocket_press_t ms5607_02ba_calculate_press(uint32_t d1, uint32_t d2) {
  rocket_press_t p;
  int32_t dt;
  int64_t off;
  int64_t sens;
  
  dt = d2 - ( prom_data.coefficient_5 * DT_MULTIPLICATIVE_OFFSET );
  off = (prom_data.coefficient_2 * OFF_MULTIPLICATIVE_OFFSET_1) + ((prom_data.coefficient_4 * dt) / OFF_MULTIPLICATIVE_OFFSET_2);
  sens = (prom_data.coefficient_1 * SENS_MULTIPLICATIVE_OFFSET_1) + ((prom_data.coefficient_3 * dt) / SENS_MULTIPLICATIVE_OFFSET_2);
  p = (((d1 * sens) / P_MULTIPLICATIVE_OFFSET_1) - off) / P_MULTIPLICATIVE_OFFSET_2;
  
  return p;
}
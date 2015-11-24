#include "ms5607_02ba.h"

#include <stdbool.h>
#include <stdint.h>

#include "asf.h"
#include "FreeRTOS.h"
#include "semphr.h"

// Device Params
#define DEVICE_ADDRESS 0x77

// Device Command Set
#define CMD_RESET             0x1E
#define CMD_CONVERTD1(osr)   (0x40 | (osr << 1))
#define CMD_CONVERTD2(osr)   (0x50 | (osr << 1))
#define CMD_ADC_READ          0x00
#define CMD_READ_REG(reg)    (0xA0 | (reg << 1))

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
#define P_MULTIPLICATIVE_OFFSET_2 32768 // 2^1

// Misc
#define SEMAPHORE_BLOCK_TIME 0

/* Private Types */
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

/* Private Function Prototypes */
static void send_command(uint8_t cmd);
static void get_data(uint8_t *buffer, uint8_t buffer_len);

/* Private Variables */
static prom_t prom_data;
static uint8_t *op_buffer;
static uint8_t op_buffer_len;
static uint8_t op_buffer_index;
static bool is_op_write;
static SemaphoreHandle_t command_complete_semaphore;
static SemaphoreHandle_t command_running_semaphore;

void ms5607_02ba_init(void) {
  TWI_MASTER_PORT.PIN0CTRL |= PORT_OPC_WIREDANDPULL_gc;
  TWI_MASTER_PORT.PIN1CTRL |= PORT_OPC_WIREDANDPULL_gc;
  
  PR.PRPE &= ~PR_TWI_bm; // Enabled TWI module clock
  
  TWI_MASTER.MASTER.BAUD = TWI_BAUD_REG;
  TWI_MASTER.MASTER.CTRLA |= TWI_MASTER_ENABLE_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_INTLVL_MED_gc;
  TWI_MASTER.MASTER.CTRLB |= TWI_MASTER_QCEN_bm | TWI_MASTER_SMEN_bm;
  TWI_MASTER.MASTER.STATUS |= TWI_MASTER_BUSSTATE_IDLE_gc;
  
  // OS Level Structures
  command_running_semaphore = xSemaphoreCreateBinary();
  if(command_running_semaphore) {
    xSemaphoreGive(command_running_semaphore);
  }
  command_complete_semaphore = xSemaphoreCreateBinary();
}

uint8_t ms5607_02ba_reset(void) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    send_command(CMD_RESET);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t ms5607_02ba_convert_D1(enum ms5607_02ba_osr osr) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    const uint8_t cmd = CMD_CONVERTD1(osr);
    send_command(cmd);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t ms5607_02ba_convert_D2(enum ms5607_02ba_osr osr) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    const uint8_t cmd = CMD_CONVERTD2(osr);
    send_command(cmd);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t ms5607_02ba_read_adc(uint32_t *adc_value) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    uint8_t buffer[3];
    send_command(CMD_ADC_READ);
    get_data(buffer, 3);
    *adc_value = ((uint32_t)buffer[0] << 8*2) | ((uint32_t)buffer[1] << 8*1) | ((uint32_t)buffer[2] << 8*0);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t ms5607_02ba_load_prom(void) { 
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    uint8_t data_buffer[2];
    
    for (uint8_t prom_addr = 0; prom_addr < 8; prom_addr++) {
      send_command(CMD_READ_REG(prom_addr));
      get_data(data_buffer, 2);
      
      *((uint16_t *)&prom_data + prom_addr) = data_buffer[0] << 8 | data_buffer[1];
    }
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

static void send_command(uint8_t cmd)
{
  op_buffer = &cmd;
  op_buffer_index = 0;
  op_buffer_len = 1;
  is_op_write = true;
  TWI_MASTER.MASTER.ADDR = DEVICE_ADDRESS << 1;
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY); // Wait for command to complete
}

static void get_data(uint8_t *buffer, uint8_t buffer_len) {
  op_buffer = buffer;
  op_buffer_index = 0;
  op_buffer_len = buffer_len;
  is_op_write = false;
  TWI_MASTER.MASTER.ADDR = DEVICE_ADDRESS << 1 | 0x1;
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY); // Wait for command to complete
}

/* Interrupts */
ISR(TWIE_TWIM_vect) {
  if(op_buffer_index < op_buffer_len) {
    TWI_MASTER.MASTER.DATA = op_buffer[op_buffer_index++];
    if(is_op_write) {
      TWI_MASTER.MASTER.DATA = op_buffer[op_buffer_index++];
    }
    else {
      op_buffer[op_buffer_index++] = TWI_MASTER.MASTER.DATA;
    }
  }
  else {
    xSemaphoreGiveFromISR(command_complete_semaphore, NULL);
    TWI_MASTER.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc; // Send stop command
  }  
}
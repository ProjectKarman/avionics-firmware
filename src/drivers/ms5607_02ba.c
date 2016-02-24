/*
 * ms5607_02ba.c
 *
 * Created: 10/03/2015 11:55:36 PM
 *  Author: Timothy Rupprecht
 */ 

#include "ms5607_02ba.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "asf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

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

#define MAG_SHIFT 1024 // 2^10

// Misc
#define SEMAPHORE_BLOCK_TIME 0
#define ASYNC_QUEUE_DEPTH 3
#define ASYNC_CMD_TYPE_MASK 0x2
#define READ_CMD_TYPE_MASK 0x1

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

enum command_type {
  CMD_TYPE_WRITE = 0x0,
  CMD_TYPE_WRITE_ASYNC = 0x2,
  CMD_TYPE_READ = 0x1,
  CMD_TYPE_READ_ASYNC = 0x3
};

/* Private Function Prototypes */
static void send_command(uint8_t cmd);
static void send_command_async(uint8_t cmd, ms5607_02ba_callback_t callback);
static void get_data(uint8_t *buffer, uint8_t buffer_len);
static void get_data_async(uint8_t read_len, ms5607_02ba_callback_t callback);
static uint32_t convert_buffer_24(uint8_t *buffer);
static uint16_t convert_buffer_16(uint8_t *buffer);
static void async_read_helper(void);

/* Private Variables */
static prom_t prom_data;
static uint8_t op_buffer[DEVICE_MAX_DATA_LEN];
static uint8_t op_buffer_len;
volatile static uint8_t op_buffer_index;
static SemaphoreHandle_t command_complete_semaphore;
static SemaphoreHandle_t command_running_semaphore;
static QueueHandle_t aysnc_data_queue;
static ms5607_02ba_callback_t current_op_callback;
static ms5607_02ba_callback_t final_op_callback;
static enum command_type current_command_type;

void ms5607_02ba_init(void) {
  TWI_MASTER_PORT.PIN0CTRL |= PORT_OPC_WIREDANDPULL_gc;
  TWI_MASTER_PORT.PIN1CTRL |= PORT_OPC_WIREDANDPULL_gc;
  
  PR.PRPE &= ~PR_TWI_bm; // Enabled TWI module clock
  
  TWI_MASTER.MASTER.BAUD = TWI_BAUD_REG;
  TWI_MASTER.MASTER.CTRLA |= TWI_MASTER_ENABLE_bm | TWI_MASTER_WIEN_bm | 
                             TWI_MASTER_RIEN_bm | TWI_MASTER_INTLVL_MED_gc;
  //TWI_MASTER.MASTER.CTRLB |= TWI_MASTER_QCEN_bm | TWI_MASTER_SMEN_bm;
  TWI_MASTER.MASTER.STATUS |= TWI_MASTER_BUSSTATE_IDLE_gc;
  
  // OS Level Structures
  aysnc_data_queue = xQueueCreate(ASYNC_QUEUE_DEPTH, sizeof(uint32_t));
  command_running_semaphore = xSemaphoreCreateBinary();
  if(command_running_semaphore) {
    xSemaphoreGive(command_running_semaphore);
  }
  command_complete_semaphore = xSemaphoreCreateBinary();
}

uint8_t ms5607_02ba_reset(void) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    send_command(CMD_RESET);
    vTaskDelay(DEVICE_RESET_DELAY);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else {
    return 1;
  }
}

uint8_t ms5607_02ba_convert_d1(enum ms5607_02ba_osr osr) {
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

uint8_t ms5607_02ba_convert_d1_async(enum ms5607_02ba_osr osr, ms5607_02ba_callback_t callback) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    const uint8_t cmd = CMD_CONVERTD1(osr);
    send_command_async(cmd, callback);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t ms5607_02ba_convert_d2(enum ms5607_02ba_osr osr) {
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

uint8_t ms5607_02ba_convert_d2_async(enum ms5607_02ba_osr osr, ms5607_02ba_callback_t callback) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    const uint8_t cmd = CMD_CONVERTD2(osr);
    send_command_async(cmd, callback);

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
    *adc_value = convert_buffer_24(buffer);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t ms5607_02ba_read_adc_async(ms5607_02ba_callback_t callback) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    final_op_callback = callback;
    send_command_async(CMD_ADC_READ, async_read_helper);

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
      
      *((uint16_t *)&prom_data + prom_addr) = convert_buffer_16(data_buffer);
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
  *op_buffer = cmd;
  op_buffer_index = 0;
  op_buffer_len = 1;
  current_command_type = CMD_TYPE_WRITE;
  TWI_MASTER.MASTER.ADDR = DEVICE_ADDRESS << 1;
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY); // Wait for command to complete
}

static void send_command_async(uint8_t cmd, ms5607_02ba_callback_t callback) {
  *op_buffer = cmd;
  op_buffer_index = 0;
  op_buffer_len = 1;
  current_command_type = CMD_TYPE_WRITE_ASYNC;
  current_op_callback = callback;
  TWI_MASTER.MASTER.ADDR = DEVICE_ADDRESS << 1;
}

static void get_data(uint8_t *buffer, uint8_t buffer_len) {
  op_buffer_index = 0;
  op_buffer_len = buffer_len;
  current_command_type = CMD_TYPE_READ;
  TWI_MASTER.MASTER.ADDR = DEVICE_ADDRESS << 1 | 0x1;
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY); // Wait for command to complete
  memcpy(buffer, op_buffer, buffer_len);
}

static void get_data_async(uint8_t read_len, ms5607_02ba_callback_t callback) {
  op_buffer_index = 0;
  op_buffer_len = read_len;
  current_command_type = CMD_TYPE_READ_ASYNC;
  current_op_callback = callback;
  TWI_MASTER.MASTER.ADDR = DEVICE_ADDRESS << 1 | 0x1;
}

rocket_temp_t ms5607_02ba_calculate_temp(uint32_t d2)	{
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
  
  return p;
}

static inline uint32_t convert_buffer_24(uint8_t *buffer) {
  return ((uint32_t)buffer[0] << 8*2) | ((uint32_t)buffer[1] << 8*1) | ((uint32_t)buffer[2] << 8*0);
}

static inline uint16_t convert_buffer_16(uint8_t *buffer) {
  return ((uint16_t)buffer[0] << 8*1) | ((uint32_t)buffer[1] << 8*0);
}

static void async_read_helper(void) {
  get_data_async(3, final_op_callback);
}

/* Interrupts */
ISR(TWIE_TWIM_vect) {
  if(!(current_command_type & READ_CMD_TYPE_MASK) && (op_buffer_index < op_buffer_len)) {
    // Write in process
    TWI_MASTER.MASTER.DATA = op_buffer[op_buffer_index++];
  }
  else if((current_command_type & READ_CMD_TYPE_MASK) && op_buffer_index < (op_buffer_len - 1)) {
    // Read in progress
    op_buffer[op_buffer_index++] = TWI_MASTER.MASTER.DATA;
    TWI_MASTER.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
  }
  else {
    if(current_command_type & READ_CMD_TYPE_MASK) {
      // Read last byte
      op_buffer[op_buffer_index++] = TWI_MASTER.MASTER.DATA;
      TWI_MASTER.MASTER.CTRLC |= TWI_MASTER_ACKACT_bm;
    }
    else {
      TWI_MASTER.MASTER.CTRLC &= ~TWI_MASTER_ACKACT_bm;
    }
    TWI_MASTER.MASTER.CTRLC |=  TWI_MASTER_CMD_STOP_gc; // Send stop command
    
    if(current_command_type & ASYNC_CMD_TYPE_MASK) {
      // Async command finished
      if(current_command_type == CMD_TYPE_READ_ASYNC) {
        // Add this data to our async read queue
        const uint32_t data = convert_buffer_24(op_buffer);
        xQueueSendToBackFromISR(aysnc_data_queue, &data, NULL);
        if(current_op_callback) {
          current_op_callback();
        }
        xSemaphoreGiveFromISR(command_running_semaphore, NULL);
      }
      else {
        if(current_op_callback) {
          current_op_callback();
        }
        xSemaphoreGiveFromISR(command_running_semaphore, NULL);
      }
    }
    else {
      // Sync command finished
      xSemaphoreGiveFromISR(command_complete_semaphore, NULL);
    }
  }
  
}
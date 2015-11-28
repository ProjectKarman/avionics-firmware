/*
 * fxls8471qr1.c
 *
 * Created: 10/08/2015 4:56:25 PM
 *  Author: Brendan
 */ 

#include "fxls8471qr1.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "asf.h"
#include "FreeRTOS.h"
#include "semphr.h"

// IO Definitions
#define SPI_CS_PIN		IOPORT_CREATE_PIN(PORTC, 4)
#define SPI_MISO_PIN	IOPORT_CREATE_PIN(PORTC, 6)
#define SPI_MOSI_PIN	IOPORT_CREATE_PIN(PORTC, 7)
#define SPI_SCLK_PIN	IOPORT_CREATE_PIN(PORTC, 5)

// registers
#define	REG_STATUS				0x00
#define REG_OUT_X_MSB			0x01
#define REG_OUT_X_LSB			0x02
#define REG_OUT_Y_MSB			0x03
#define REG_OUT_Y_LSB			0x04
#define REG_OUT_Z_MSB			0x05
#define REG_OUT_Z_LSB			0x06
#define REG_F_SETUP				0x09
#define REG_XYZ_DATA_CFG		0x0E
#define REG_F_STATUS			0x10
#define REG_CTRL_REG1			0x2A
#define REG_CTRL_REG2			0x2B
#define REG_CTRL_REG3			0x2C
#define REG_CTRL_REG4			0x2D
#define REG_CTRL_REG5			0x2E

// commands
#define SPICMD_W_REGISTER_MSB(reg) (0x80 | reg)
#define SPICMD_W_REGISTER_LSB(reg) (reg)
#define SPICMD_R_REGISTER_MSB(reg) (reg & ~0x80)
#define SPICMD_R_REGISTER_LSB(reg) (reg)

// USART Peripheral
#define SPI_CNTL USARTC1
#define F_PER 32000000
#define DESIRED_BITRATE 10000000
#define USART_BSEL (F_PER / (2 * DESIRED_BITRATE) - 1)

// Misc
#define OP_BUFFER_LEN 8
#define DATA_READ_LENGTH 7
#define SEMAPHORE_BLOCK_TIME 0
#define ASYNC_CMD_TYPE_MASK 0x2
#define READ_CMD_TYPE_MASK 0x1
#define DUMMY_DATA 0xff

/* Private Types */
enum command_type {
  CMD_TYPE_WRITE = 0x0,
  CMD_TYPE_WRITE_ASYNC = 0x2,
  CMD_TYPE_READ = 0x1,
  CMD_TYPE_READ_ASYNC = 0x3
};

/* Private Function Prototypes */
static void spi_startframe(void);
static void spi_endframe(void);
static void write_register_single(uint8_t address, uint8_t values);
static void write_register(uint8_t address, uint8_t *values, uint8_t len);
static void write_register_async(uint8_t address, uint8_t *buffer, uint8_t len, fxls8471qr1_callback_t callback);
static void read_register_single(uint8_t address, uint8_t *buffer);
static void read_register(uint8_t address, uint8_t *buffer, uint8_t len);
static void read_register_async(uint8_t address, uint8_t *buffer, uint8_t len, fxls8471qr1_data_callback_t callback);

/* Private Variables */
//static uint8_t local_fifo_config;
static union {
  struct {
    uint8_t f_wmrk : 6;
    uint8_t f_mode : 2;
  };
  uint8_t raw;
} local_f_setup;
static union {
  struct {
    uint8_t fs : 2;
    uint8_t pad0 : 2;
    uint8_t hpf_out : 1;
  };
  uint8_t raw;
} local_xyz_data_config;
static union {
  struct {
    uint8_t active : 1;
    uint8_t f_read : 1;
    uint8_t lnoise : 1;
    uint8_t dr : 3;
    uint8_t aslp_rate : 2;
  };
  uint8_t raw;  
} local_ctrl_reg1_config;
static uint8_t local_ctrl_reg4_config;
static uint8_t local_ctrl_reg5_config;
static fxls8471qr1_callback_t write_function_callback;
static fxls8471qr1_data_callback_t read_function_callback;
static uint8_t op_buffer[OP_BUFFER_LEN];
static uint8_t op_len;
static uint8_t op_buffer_index;
static enum command_type op_type;
static SemaphoreHandle_t command_complete_semaphore;
static SemaphoreHandle_t command_running_semaphore;

/* Public Functions */
void fxls8471qr1_init()
{
  // Config GPIO
	ioport_set_pin_dir(SPI_CS_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SPI_MISO_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SPI_MOSI_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SPI_SCLK_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_HIGH);
  
  // Configure USART
  sysclk_enable_module(SYSCLK_PORT_C, PR_USART1_bm);
  SPI_CNTL.CTRLC |= USART_CMODE_MSPI_gc;
  SPI_CNTL.BAUDCTRLA = USART_BSEL;
  
  // Config OS Level Structures
  command_running_semaphore = xSemaphoreCreateBinary();
  if(command_running_semaphore) {
    xSemaphoreGive(command_running_semaphore);
  }
  command_complete_semaphore = xSemaphoreCreateBinary();
}

uint8_t fxls8471qr1_set_fifo_mode(enum fxls8471qr1_fifo_mode fifo_mode)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_f_setup.f_mode = fifo_mode;
    write_register_single(REG_F_SETUP, local_f_setup.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}  

uint8_t fxls8471qr1_set_fifo_watermark(uint8_t watermark)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_f_setup.f_wmrk = watermark;
    write_register_single(REG_F_SETUP, local_f_setup.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t fxls8471qr1_set_fs_range(enum fxls8471qr1_fs_range fs_range)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_xyz_data_config.fs = fs_range;
    write_register_single(REG_F_SETUP, local_xyz_data_config.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t fxls8471qr1_set_hp_filter(bool is_enabled)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_xyz_data_config.hpf_out = is_enabled;
    write_register_single(REG_XYZ_DATA_CFG, local_xyz_data_config.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t fxls8471qr1_set_data_rate(enum fxls8471qr1_data_rate data_rate)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_ctrl_reg1_config.dr = data_rate;
    write_register_single(REG_CTRL_REG1, local_ctrl_reg1_config.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t fxls8471qr1_set_interrupts(fxls8471qr1_bitfield_t bitfield)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_ctrl_reg4_config = bitfield;
    write_register_single(REG_CTRL_REG4, local_ctrl_reg4_config);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

/*
 * Set interrupt flags will use INT1 pin, other will use INT2
 */
uint8_t fxls8471qr1_set_interrupt_route(fxls8471qr1_bitfield_t bitfield)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_ctrl_reg5_config = bitfield;
    write_register_single(REG_CTRL_REG5, local_ctrl_reg5_config);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t fxls8471qr1_get_data(fxls8471qr1_raw_accel_t *data) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    uint8_t buffer[DATA_READ_LENGTH];
    read_register(REG_STATUS, buffer, DATA_READ_LENGTH);
    data->x = buffer[REG_OUT_X_MSB] << 6 | buffer[REG_OUT_X_LSB];
    data->y = buffer[REG_OUT_Y_MSB] << 6 | buffer[REG_OUT_Y_LSB];
    data->z = buffer[REG_OUT_Z_MSB] << 6 | buffer[REG_OUT_Z_LSB];
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}



/* Private Functions */
static inline void spi_startframe(void){
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_LOW);
}

static inline void spi_endframe(void){
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_HIGH);
}

static inline void write_register_single(uint8_t address, uint8_t values) {
  write_register(address, &values, 1);
}

static void write_register(uint8_t address, uint8_t *buffer, uint8_t len) {
  op_buffer[0] = SPICMD_W_REGISTER_MSB(address);
  op_buffer[1] = SPICMD_W_REGISTER_LSB(address);
  memcpy(op_buffer + 2, buffer, len);
  op_len = len + 2;
  op_buffer_index = 0;
  op_type = CMD_TYPE_WRITE;
  
  SPI_CNTL.CTRLA |= USART_DREINTLVL_MED_gc | USART_TXCINTLVL_MED_gc;
  spi_startframe();
  SPI_CNTL.DATA = op_buffer[0];
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY); // Wait for command to complete
}

static void write_register_async(uint8_t address, uint8_t *buffer, uint8_t len, fxls8471qr1_callback_t callback) {
  write_function_callback = callback;
  op_buffer[0] = SPICMD_W_REGISTER_MSB(address);
  op_buffer[1] = SPICMD_W_REGISTER_LSB(address);
  memcpy(op_buffer + 2, buffer, len);
  op_len = len + 2;
  op_buffer_index = 0;
  op_type = CMD_TYPE_WRITE_ASYNC;
  
  SPI_CNTL.CTRLA |= USART_DREINTLVL_MED_gc | USART_TXCINTLVL_MED_gc;
  spi_startframe();
  SPI_CNTL.DATA = op_buffer[0];
}

static inline void read_register_single(uint8_t address, uint8_t *buffer) {
  read_register(address, buffer, 1);
}

static void read_register(uint8_t address, uint8_t *buffer, uint8_t len) {
  op_buffer[0] = SPICMD_R_REGISTER_MSB(address);
  op_buffer[1] = SPICMD_R_REGISTER_LSB(address);
  op_len = len + 2;
  op_buffer_index = 0;
  op_type = CMD_TYPE_READ;
  
  SPI_CNTL.CTRLA |= USART_DREINTLVL_MED_gc | USART_TXCINTLVL_MED_gc;
  spi_startframe();
  SPI_CNTL.DATA = op_buffer[0];
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY); // Wait for command to complete
}

static void read_register_async(uint8_t address, uint8_t *buffer, uint8_t len, fxls8471qr1_data_callback_t callback) {
  read_function_callback = callback;
  op_buffer[0] = SPICMD_R_REGISTER_MSB(address);
  op_buffer[1] = SPICMD_R_REGISTER_LSB(address);
  op_len = len + 2;
  op_buffer_index = 0;
  op_type = CMD_TYPE_READ_ASYNC;
  
  SPI_CNTL.CTRLA |= USART_DREINTLVL_MED_gc | USART_TXCINTLVL_MED_gc;
  spi_startframe();
  SPI_CNTL.DATA = op_buffer[0];
}

ISR(USARTC1_TXC_vect) {
  if(op_type & ASYNC_CMD_TYPE_MASK) {
    // Async Command
    spi_endframe();
    xSemaphoreGiveFromISR(command_running_semaphore, NULL);
    if((op_type & READ_CMD_TYPE_MASK) && read_function_callback) {
      /*
       * Here we are assuming that the read async functionality will only
       * be used to read data off the accel
       */
      fxls8471qr1_raw_accel_t data;
      data.x = op_buffer[REG_OUT_X_MSB] << 6 | op_buffer[REG_OUT_X_LSB];
      data.y = op_buffer[REG_OUT_Y_MSB] << 6 | op_buffer[REG_OUT_Y_LSB];
      data.z = op_buffer[REG_OUT_Z_MSB] << 6 | op_buffer[REG_OUT_Z_LSB];
      read_function_callback(data); // Read Async Call
    }
    else if(write_function_callback) {
      write_function_callback(); // Write Async Call
    }
  }
  else {
    // Sync Command
    spi_endframe();
    xSemaphoreGiveFromISR(command_complete_semaphore, NULL);
  }
}

ISR(USARTC1_DRE_vect) {
  op_buffer_index++;
  if(op_buffer_index == 1) {
    // Send Command LSB
    SPI_CNTL.DATA = op_buffer[op_buffer_index];
  }
  else {
    if(op_type & READ_CMD_TYPE_MASK) {
      // Read Command
      if(op_buffer_index == 2) {
        // Flush FIFO
        SPI_CNTL.DATA;
        SPI_CNTL.DATA;
        SPI_CNTL.DATA = DUMMY_DATA;
      }
      else if(op_buffer_index < op_len) {
        // Continue Read
        op_buffer[op_buffer_index - 2] = SPI_CNTL.DATA;
        SPI_CNTL.DATA = DUMMY_DATA;
      }
      else {
        // End Read
        op_buffer[op_buffer_index - 2] = SPI_CNTL.DATA;
        SPI_CNTL.STATUS |= USART_DREIF_bm;
      }
    }
    else {
      // Write Command
      if(op_buffer_index < op_len) {
        SPI_CNTL.DATA = op_buffer[op_buffer_index];
      }
      else {
        SPI_CNTL.STATUS |= USART_DREIF_bm;
      }
    }  
  }    
}
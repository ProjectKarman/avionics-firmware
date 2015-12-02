/*
 * l3g4200d.c
 *
 * Created: 11/8/2015 10:45:10 PM
 *  Author: Brendan
 */ 

#include "l3g4200d.h"

#include <string.h>
#include <stdbool.h>
#include <string.h>

#include "asf.h"
#include "FreeRTOS.h"
#include "semphr.h"

// IO definitions (NEED CORRECT PORTS)
#define SPI_CS_PIN IOPORT_CREATE_PIN(PORTC, 4)
#define SPI_MISO_PIN IOPORT_CREATE_PIN(PORTC, 6)
#define SPI_MOSI_PIN IOPORT_CREATE_PIN(PORTC, 7)
#define SPI_SCLK_PIN IOPORT_CREATE_PIN(PORTC, 5)
#define INT1_PIN IOPORT_CREATE_PIN(PORTC, 0)
#define INT2_PIN IOPORT_CREATE_PIN(PORTC, 1)

//registers
#define REG_WHO_AM_I 0x0F
#define REG_CTRL_REG1 0x20
#define REG_CTRL_REG2 0x21
#define REG_CTRL_REG3 0x22
#define REG_CTRL_REG4 0x23
#define REG_CTRL_REG5 0x24
#define REG_REFERENCE 0x25
#define REG_OUT_TEMP 0x26
#define REG_STATUS_REG 0x27
#define REG_OUT_X_L 0x28
#define REG_OUT_X_H 0x29
#define REG_OUT_Y_L 0x2A
#define REG_OUT_Y_H 0x2B
#define REG_OUT_Z_L 0x2C
#define REG_OUT_Z_H 0x2D
#define REG_FIFO_CTRL_REG 0x2E
#define REG_FIFO_SRC_REG 0x2F
#define REG_INT1_CFG 0x30
#define REG_INT1_SRC 0x31
#define REG_INT1_TSH_XH 0x32
#define REG_INT1_TSH_XL 0x33
#define REG_INT1_TSH_YH 0x34
#define REG_INT1_TSH_YL 0x35
#define REG_INT1_TSH_ZH 0x36
#define REG_INT1_TSH_ZL 0x37
#define REG_INT1_DURATION 0x38

//commands
#define SPICMD_W_REGISTER(reg) (reg & 0x3F)
#define SPICMD_W_REGISTER_INC(reg) ((reg & 0x3F) | 0x40)
#define SPICMD_R_REGISTER(reg) (reg | 0x80)
#define SPICMD_R_REGISTER_INC(reg) (reg | 0xC0)
#define SPICMD_LEN 1

// peripheral definitions
#define SPI_CTRL USARTC1
#define F_PER 32000000
#define DESIRED_BITRATE 10000000
#define USART_BSEL 15 // (F_PER / (2 * DESIRED_BITRATE) - 1)

// misc
#define OP_BUFFER_LEN 8
#define DATA_READ_LENGTH 7
#define SEMAPHORE_BLOCK_TIME 0
#define ASYNC_CMD_TYPE_MASK 0x2
#define READ_CMD_TYPE_MASK 0x1
#define DUMMY_DATA 0xFF

// Private types
enum command_type {
  CMD_TYPE_WRITE = 0x0,
  CMD_TYPE_WRITE_ASYNC = 0x2,
  CMD_TYPE_READ = 0x1,
  CMD_TYPE_READ_ASYNC = 0x3
};

// Private functions
static void spi_startframe(void);
static void spi_endframe(void);
static void raw_xyz_pack_from_buffer(l3g4200d_raw_xyz_t *raw_xyz, uint8_t *buffer);
static void write_register_single(uint8_t address, uint8_t values);
static void write_register(uint8_t address, uint8_t *values, uint8_t len);
static void write_register_async(uint8_t address, uint8_t *buffer, uint8_t len, l3g4200d_callback_t callback);
static void read_register_single(uint8_t address);
static void read_register(uint8_t address, uint8_t len);
static void read_register_async(uint8_t address, uint8_t len, l3g4200d_data_callback_t callback);

// Private variables
static union {
  struct {
    uint8_t f_wmrk: 3;
    uint8_t f_mode: 5;
  };
  uint8_t raw;
} local_fifo_ctrl_reg_config;
static union {
  struct {
    uint8_t xen: 1;
    uint8_t yen: 1;
    uint8_t zen: 1;
    uint8_t p_mode: 1;
    uint8_t odr_bw: 4;
  };
  uint8_t raw;
} local_ctrl_reg1_config;
static union {
  struct {
    uint8_t hp_cutoff: 4;
    uint8_t hp_mode: 2;
  };
  uint8_t raw;
} local_ctrl_reg2_config;
static union {
  struct {
    uint8_t bdu: 1;
    uint8_t bl_endian: 1;
    uint8_t fs_sel: 2;
    uint8_t pad0: 1;
    uint8_t self_test_en: 2;
    uint8_t spi_mode: 1;
  };
  uint8_t raw;
} local_ctrl_reg4_config;
static union {
  struct {
    uint8_t boot_mode: 1;
    uint8_t fifo_en: 1;
    uint8_t pad0: 1;
    uint8_t hp_en: 1;
    uint8_t int_sel: 2;
    uint8_t out_sel: 2;
  };
  uint8_t raw;
} local_ctrl_reg5_config;
static union {
  struct {
    uint8_t wait: 1;
    uint8_t d_val: 7;
  };
  uint8_t raw;
} local_int1_duration_config;

static uint8_t local_ctrl_reg3_config;
static uint8_t local_int1_cfg_config;
static l3g4200d_callback_t int1_pin_callback;
static l3g4200d_callback_t int2_pin_callback;
static l3g4200d_callback_t write_function_callback;
static l3g4200d_data_callback_t read_function_callback;
static uint8_t op_buffer[OP_BUFFER_LEN];
static uint8_t op_len;
static uint8_t op_buffer_index;
static enum command_type op_type;
static SemaphoreHandle_t command_complete_semaphore;
static SemaphoreHandle_t command_running_semaphore;

void l3g4200d_init() {
  ioport_set_pin_dir(SPI_CS_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(SPI_MISO_PIN, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(SPI_MOSI_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(SPI_SCLK_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_HIGH);
  
  // configure interrupt pins
  ioport_set_pin_sense_mode(INT1_PIN, IOPORT_SENSE_FALLING);
  arch_ioport_pin_to_base(INT1_PIN)->INT0MASK |= arch_ioport_pin_to_mask(INT1_PIN);
  arch_ioport_pin_to_base(INT1_PIN)->INTCTRL |= PORT_INT0LVL_MED_gc;
  ioport_set_pin_sense_mode(INT2_PIN, IOPORT_SENSE_FALLING);
  arch_ioport_pin_to_base(INT2_PIN)->INT0MASK |= arch_ioport_pin_to_mask(INT2_PIN);
  arch_ioport_pin_to_base(INT2_PIN)->INTCTRL |= PORT_INT0LVL_MED_gc;

  //configure USART
  sysclk_enable_module(SYSCLK_PORT_C, PR_USART1_bm);
  SPI_CTRL.CTRLA |= USART_TXCINTLVL_MED_gc;
  SPI_CTRL.CTRLB |= USART_TXEN_bm | USART_RXEN_bm;
  SPI_CTRL.CTRLC |= USART_CMODE_MSPI_gc;
  SPI_CTRL.CTRLC &= ~(USART_CHSIZE2_bm | USART_CHSIZE2_bm);
  SPI_CTRL.BAUDCTRLA = USART_BSEL;

  // Configure OS level structures
  command_running_semaphore = xSemaphoreCreateBinary();
  if (command_running_semaphore) {
    xSemaphoreGive(command_running_semaphore);
  }
  command_complete_semaphore = xSemaphoreCreateBinary();
}

// Private Functions

static inline void spi_startframe(void) {
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_LOW);
}

static inline void spi_endframe(void) {
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_HIGH);
}

static inline void raw_xyz_pack_from_buffer(l3g4200d_raw_xyz_t *raw_xyz, uint8_t *buffer) {
  raw_xyz->x = (uint16_t)buffer[REG_OUT_X_H] << 8 | (uint16_t)buffer[REG_OUT_X_L];
  raw_xyz->y = (uint16_t)buffer[REG_OUT_Y_H] << 8 | (uint16_t)buffer[REG_OUT_Y_L];
  raw_xyz->z = (uint16_t)buffer[REG_OUT_Z_H] << 8 | (uint16_t)buffer[REG_OUT_Z_L];
}

static inline void write_register_single(uint8_t address, uint8_t values){
  write_register(address, &values, 1);
}

static void write_register(uint8_t address, uint8_t *buffer, uint8_t len){
  //op_buffer[0] = SPICMD_W_REGISTER(address);
  memcpy(op_buffer, buffer, len);
  op_len = len + SPICMD_LEN;
  op_buffer_index = 0;
  op_type = CMD_TYPE_WRITE;
  
  spi_startframe();
  SPI_CTRL.DATA = SPICMD_W_REGISTER(address);
  //while(!(SPI_CTRL.STATUS & USART_DREIF_bm));
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY);
}

static void write_register_async(uint8_t address, uint8_t *buffer, uint8_t len, l3g4200d_callback_t callback){
  write_function_callback = callback;
  op_buffer[0] = SPICMD_W_REGISTER(address);
  memcpy(op_buffer + 1, buffer, len);
  op_len = len + SPICMD_LEN;
  op_buffer_index = 0;
  op_type = CMD_TYPE_WRITE_ASYNC;
  
   spi_startframe();
   SPI_CTRL.DATA = SPICMD_W_REGISTER(address);
   //while(!(SPI_CTRL.STATUS & USART_DREIF_bm));
}

static inline void read_register_single(uint8_t address){
  read_register(address, 1);
}

static void read_register(uint8_t address, uint8_t len){
  op_buffer[0] = SPICMD_R_REGISTER(address);
  memset(op_buffer, DUMMY_DATA, len);
  op_len = len + SPICMD_LEN;
  op_buffer_index = 0;
  op_type = CMD_TYPE_READ;
  
  spi_startframe();
  SPI_CTRL.DATA = SPICMD_R_REGISTER(address);
  //while(!(SPI_CTRL.STATUS & USART_DREIF_bm));
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY);
}

static void read_register_async(uint8_t address, uint8_t len, l3g4200d_data_callback_t callback){
  write_function_callback = callback;
  op_buffer[0] = SPICMD_R_REGISTER(address);
  op_len = len + SPICMD_LEN;
  op_buffer_index = 0;
  op_type = CMD_TYPE_READ_ASYNC;
  
  spi_startframe();
  SPI_CTRL.DATA = SPICMD_R_REGISTER(address);
  //while(!(SPI_CTRL.STATUS & USART_DREIF_bm));
}
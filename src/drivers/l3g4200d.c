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

// IO definitions
#define SPI_CS_PIN IOPORT_CREATE_PIN(PORTE, 5)
#define SPI_MISO_PIN IOPORT_CREATE_PIN(PORTD, 6)
#define SPI_MOSI_PIN IOPORT_CREATE_PIN(PORTD, 7)
#define SPI_SCLK_PIN IOPORT_CREATE_PIN(PORTD, 5)
#define INT1_PIN IOPORT_CREATE_PIN(PORTE, 6)
#define INT2_PIN IOPORT_CREATE_PIN(PORTE, 7)

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


#define WHO_AM_I_RESPONSE 0xD3

//commands
#define SPICMD_W_REGISTER(reg) (reg & 0x3F)
#define SPICMD_W_REGISTER_INC(reg) ((reg & 0x3F) | 0x40)
#define SPICMD_R_REGISTER(reg) (reg | 0x80)
#define SPICMD_R_REGISTER_INC(reg) (reg | 0xC0)
#define SPICMD_THS_H_REG(reg) ((uint8_t)(reg >> 8))
#define SPICMD_THS_L_REG(reg) ((uint8_t)reg)
#define SPICMD_INT1_DURATION(reg) (reg & 0x7F)
#define SPICMD_LEN 1

// peripheral definitions
#define SPI_CTRL USARTD1
#define F_PER 32000000
#define DESIRED_BITRATE 10000000
#define USART_BSEL 16 // (F_PER / (2 * DESIRED_BITRATE) - 1)

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
    uint8_t f_wmrk: 5;
    uint8_t f_mode: 3;
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
    uint8_t s_mode: 1;
  };
  uint8_t raw;
} local_ctrl_reg4_config;
static union {
  struct {
    uint8_t boot_mode: 1;
    uint8_t fifo_en: 1;
    uint8_t pad0: 1;
    uint8_t hp_en: 1;
    uint8_t int1_sel: 2;
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
static uint16_t local_int1_ths_x_config;
static uint16_t local_int1_ths_y_config;
static uint16_t local_int1_ths_z_config;
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
  sysclk_enable_module(SYSCLK_PORT_D, PR_USART1_bm);
  SPI_CTRL.CTRLA |= USART_RXCINTLVL_MED_gc;
  SPI_CTRL.CTRLB |= USART_TXEN_bm | USART_RXEN_bm;
  SPI_CTRL.CTRLC |= USART_CMODE_MSPI_gc;
  SPI_CTRL.CTRLC &= ~(USART_CHSIZE2_bm | USART_CHSIZE1_bm);
  SPI_CTRL.BAUDCTRLA = USART_BSEL;

  // Configure OS level structures
  command_running_semaphore = xSemaphoreCreateBinary();
  if (command_running_semaphore) {
    xSemaphoreGive(command_running_semaphore);
  }
  command_complete_semaphore = xSemaphoreCreateBinary();
}

// Public Functions
uint8_t l3g4200d_check_comms(void){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    read_register_single(REG_WHO_AM_I);
    xSemaphoreGive(command_running_semaphore);
    if(op_buffer[0] == WHO_AM_I_RESPONSE){
      return 0;
    }
    else {
      return 2;
    }
  }
  else {
    return 1;
  }
}

uint8_t l3g4200d_setup_fifo_mode(enum l3g4200d_fifo_mode fifo_mode){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_fifo_ctrl_reg_config.f_mode = fifo_mode;
    write_register_single(REG_FIFO_CTRL_REG, local_fifo_ctrl_reg_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_fifo_watermark(uint8_t watermark){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_fifo_ctrl_reg_config.f_wmrk = watermark;
    write_register_single(REG_FIFO_CTRL_REG, local_fifo_ctrl_reg_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg1_dr_bw(enum l3g4200d_ctrl_reg1_odr_bw odr_bw_select){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg1_config.odr_bw = odr_bw_select;
    write_register_single(REG_CTRL_REG1, local_ctrl_reg1_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg2_hp(enum l3g4200d_ctrl_reg2_hp_mode hp_mode){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg2_config.hp_mode = hp_mode;
    write_register_single(REG_CTRL_REG2, local_ctrl_reg2_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg2_hp_cutoff(enum l3g4200d_ctrl_reg2_hp_cutoff hp_cutoff){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg2_config.hp_cutoff = hp_cutoff;
    write_register_single(REG_CTRL_REG2, local_ctrl_reg2_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg3(l3g4200d_bitfield_t bitfield){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg3_config = bitfield;
    write_register_single(REG_CTRL_REG3, local_ctrl_reg3_config);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg4_fs_select(enum l3g4200d_ctrl_reg4_fs_select fs_select){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg4_config.fs_sel = fs_select;
    write_register_single(REG_CTRL_REG4, local_ctrl_reg4_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg4_self_test_enable(enum l3g4200d_ctrl_reg4_self_test_enable self_test){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg4_config.self_test_en = self_test;
    write_register_single(REG_CTRL_REG4, local_ctrl_reg4_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg4_spi_mode_select(enum l3g4200d_ctrl_reg4_spi_mode spi_mode){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg4_config.s_mode = spi_mode;
    write_register_single(REG_CTRL_REG4, local_ctrl_reg4_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg5_fifo_enable(bool enable){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg5_config.fifo_en = enable;
    write_register_single(REG_CTRL_REG5, local_ctrl_reg5_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg5_hp_filter_enable(bool enable){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg5_config.hp_en = enable;
    write_register_single(REG_CTRL_REG5, local_ctrl_reg5_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg5_int1_sel(enum l3g4200d_ctrl_reg5_int1_sel int1_sel){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg5_config.int1_sel = int1_sel;
    write_register_single(REG_CTRL_REG5, local_ctrl_reg5_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_ctrl_reg5_out_sel(enum l3g4200d_ctrl_reg5_out_sel out_sel){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg5_config.out_sel = out_sel;
    write_register_single(REG_CTRL_REG5, local_ctrl_reg5_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_int1_cfg(l3g4200d_bitfield_t bitfield){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_int1_cfg_config = bitfield;
    write_register_single(REG_INT1_CFG, local_int1_cfg_config);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_int1_ths_x(uint16_t threshold){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_int1_ths_x_config = threshold;
    write_register_single(REG_INT1_TSH_XH, SPICMD_THS_H_REG(local_int1_ths_x_config));
    write_register_single(REG_INT1_TSH_XL, SPICMD_THS_L_REG(local_int1_ths_x_config));
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_int1_ths_y(uint16_t threshold){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_int1_ths_y_config = threshold;
    write_register_single(REG_INT1_TSH_YH, SPICMD_THS_H_REG(local_int1_ths_y_config));
    write_register_single(REG_INT1_TSH_YL, SPICMD_THS_L_REG(local_int1_ths_y_config));
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_int1_ths_z(uint16_t threshold){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_int1_ths_z_config = threshold;
    write_register_single(REG_INT1_TSH_ZH, SPICMD_THS_H_REG(local_int1_ths_z_config));
    write_register_single(REG_INT1_TSH_ZL, SPICMD_THS_L_REG(local_int1_ths_z_config));
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_int1_wait_enable(bool enable){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_int1_duration_config.wait = enable;
    write_register_single(REG_INT1_DURATION, local_int1_duration_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_setup_int1_duration(l3g4200d_bitfield_t bitfield){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_int1_duration_config.d_val = SPICMD_INT1_DURATION(bitfield);
    write_register_single(REG_INT1_CFG, local_int1_duration_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}

uint8_t l3g4200d_get_data(l3g4200d_raw_xyz_t *data){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    uint8_t buffer[DATA_READ_LENGTH];
    read_register_single(REG_OUT_X_L);
    buffer[0] = op_buffer[1];
    read_register_single(REG_OUT_X_H);
    buffer[1] = op_buffer[1];
    read_register_single(REG_OUT_Y_L);
    buffer[2] = op_buffer[1];
    read_register_single(REG_OUT_Y_H);
    buffer[3] = op_buffer[1];
    read_register_single(REG_OUT_Z_L);
    buffer[4] = op_buffer[1];
    read_register_single(REG_OUT_Z_H);
    buffer[5] = op_buffer[1];
    raw_xyz_pack_from_buffer(data, buffer);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else{
    return 1;
  }
}
//CURRENTLY DOES NOT WORK
uint8_t l3g4200d_get_data_async(l3g4200d_data_callback_t callback){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    read_register_async(REG_STATUS_REG, DATA_READ_LENGTH, callback);
    
    return 0;
  }
  else{
    return 1;
  }
}
//CURRENTLY DOES NOT WORK
uint8_t l3g4200d_get_data_async_from_isr(l3g4200d_data_callback_t callback){
  if(xSemaphoreTakeFromISR(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    read_register_async(REG_STATUS_REG, DATA_READ_LENGTH, callback);
    
    return 0;
  }
  else {
    return 1;
  }
}

uint8_t l3g4200d_activate(void){
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE){
    local_ctrl_reg1_config.p_mode = 0x1;
    local_ctrl_reg1_config.xen = 0x1;
    local_ctrl_reg1_config.yen = 0x1;
    local_ctrl_reg1_config.zen = 0x1;
    write_register_single(REG_CTRL_REG1, local_ctrl_reg1_config.raw);
    xSemaphoreGive(command_running_semaphore);
    
    return 0;
  }
  else {
    return 1;
  }
}

void l3g4200d_set_int1_handler(l3g4200d_callback_t callback){
  int1_pin_callback = callback;
}

void l3g4200d_set_int2_handler(l3g4200d_callback_t callback){
  int2_pin_callback = callback;
}

// Private Functions

static inline void spi_startframe(void) {
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_LOW);
}

static inline void spi_endframe(void) {
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_HIGH);
}

static inline void raw_xyz_pack_from_buffer(l3g4200d_raw_xyz_t *raw_xyz, uint8_t *buffer) {
  raw_xyz->x = (uint16_t)buffer[1] << 8 | (uint16_t)buffer[0];
  raw_xyz->y = (uint16_t)buffer[3] << 8 | (uint16_t)buffer[1];
  raw_xyz->z = (uint16_t)buffer[5] << 8 | (uint16_t)buffer[2];
}

static inline void write_register_single(uint8_t address, uint8_t values){
  write_register(address, &values, 1);
}

static void write_register(uint8_t address, uint8_t *buffer, uint8_t len){
  memcpy(op_buffer, buffer, len);
  op_len = len + SPICMD_LEN;
  op_buffer_index = 1;
  op_type = CMD_TYPE_WRITE;
  
  spi_startframe();
  SPI_CTRL.DATA = SPICMD_W_REGISTER(address);
  while(!(SPI_CTRL.STATUS & USART_DREIF_bm));
  SPI_CTRL.DATA = buffer[0];
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY);
}

static void write_register_async(uint8_t address, uint8_t *buffer, uint8_t len, l3g4200d_callback_t callback){
  write_function_callback = callback;
  memcpy(op_buffer + 1, buffer, len);
  op_len = len + SPICMD_LEN;
  op_buffer_index = 1;
  op_type = CMD_TYPE_WRITE_ASYNC;
  
   spi_startframe();
   SPI_CTRL.DATA = SPICMD_W_REGISTER(address);
   while(!(SPI_CTRL.STATUS & USART_DREIF_bm));
   SPI_CTRL.DATA = buffer[0];
}

static inline void read_register_single(uint8_t address){
  read_register(address, 1);
}

static void read_register(uint8_t address, uint8_t len){
  op_buffer[0] = SPICMD_R_REGISTER(address);
  memset(op_buffer, DUMMY_DATA, len);
  op_len = len + SPICMD_LEN;
  op_buffer_index = 1;
  op_type = CMD_TYPE_READ;
  
  spi_startframe();
  SPI_CTRL.DATA = SPICMD_R_REGISTER_INC(address);
  while(!(SPI_CTRL.STATUS & USART_DREIF_bm));
  SPI_CTRL.DATA = DUMMY_DATA;
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY);
}

static void read_register_async(uint8_t address, uint8_t len, l3g4200d_data_callback_t callback){
  write_function_callback = callback;
  op_buffer[0] = SPICMD_R_REGISTER(address);
  op_len = len + SPICMD_LEN;
  op_buffer_index = 1;
  op_type = CMD_TYPE_READ_ASYNC;
  
  spi_startframe();
  SPI_CTRL.DATA = SPICMD_R_REGISTER(address);
  while(!(SPI_CTRL.STATUS & USART_DREIF_bm));
  SPI_CTRL.DATA = DUMMY_DATA;
}

ISR(USARTD1_RXC_vect) {
  if(op_buffer_index + 2 < op_len) {
    SPI_CTRL.DATA = op_buffer[op_buffer_index];
  }    
  if(op_buffer_index <= 1) {
    op_buffer[0] = SPI_CTRL.DATA;
  }
  else if(op_buffer_index + 1 < op_len) {
    op_buffer[op_buffer_index - 1] = SPI_CTRL.DATA;
  }
  else {
    op_buffer[op_buffer_index - 1] = SPI_CTRL.DATA;
    if(op_type & ASYNC_CMD_TYPE_MASK) {
      spi_endframe();
      xSemaphoreGiveFromISR(command_running_semaphore, NULL);
      if((op_type & READ_CMD_TYPE_MASK) && read_function_callback){
        l3g4200d_raw_xyz_t data;
        raw_xyz_pack_from_buffer(&data, op_buffer);
        read_function_callback(data);
      }
      else if(write_function_callback){
        write_function_callback();
      }
    }
    else {
      spi_endframe();
      xSemaphoreGiveFromISR(command_complete_semaphore, NULL);
    }
  }
  op_buffer_index++;
}

ISR(PORTE_INT0_vect) {
  if(PORTE.IN & ioport_pin_to_mask(INT1_PIN)){
    if(int1_pin_callback){
      int1_pin_callback();
    }
  }
  else if(PORTE.IN & ioport_pin_to_mask(INT2_PIN)){
    if(int2_pin_callback){
      int2_pin_callback();
    }
  }
}
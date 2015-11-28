/*
 * fxls8471qr1.c
 *
 * Created: 10/08/2015 4:56:25 PM
 *  Author: Brendan
 */ 

#include "fxls8471qr1.h"

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

// data read length for accel
#define DATA_READ_LENGTH		6

// commands
#define SPICMD_W_REGISTER_MSB(reg) (0x80 | reg)
#define SPICMD_W_REGISTER_LSB(reg) (reg)
#define SPICMD_R_REGISTER_MSB(reg) (reg & ~0x80)
#define SPICMD_R_REGISTER_LSB(reg) (reg)

// FIFO setup reg bits
#define F_SETUP_FIFO_OFF 0x00
#define F_SETUP_FIFO_CIRCULAR_BUFFER 0x40
#define F_SETUP_FIFO_STOP_ACCEPTING 0x80
#define F_SETUP_FIFO_TRIGGER 0xC0

// DR setup reg bits
#define DR_800_HZ 0x00
#define DR_400_HZ 0x08
#define DR_200_HZ 0x10
#define DR_100_HZ 0x18
#define DR_50_HZ 0x20
#define DR_12_5_HZ 0x28
#define DR_6_25_HZ 0x30
#define DR_1_5625_HZ 0x38

//fs range bits
#define FS_RANGE_244 0x00
#define FS_RANGE_488 0x01
#define FS_RANGE_976 0x02

//Each bit is on
#define EIGHTH_BIT_ON 0x80
#define SEVENTH_BIT_ON 0x40
#define SIXTH_BIT_ON 0x20
#define FIFTH_BIT_ON 0x10
#define FOURTH_BIT_ON 0x08
#define THIRD_BIT_ON 0x04
#define SECOND_BIT_ON 0x02
#define FIRST_BIT_ON 0x01


// USART Peripheral
#define SPI_CNTL USARTC1
#define F_PER 32000000
#define DESIRED_BITRATE 10000000
#define USART_BSEL (F_PER / (2 * DESIRED_BITRATE) - 1)

// Misc
#define OP_BUFFER_LEN 8
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
static uint8_t local_fifo_config;
static uint8_t local_xyz_data_config;
static uint8_t local_ctrl_reg1_config;
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
void fxls8471qr1_init() {
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

/*
void fxls8471qr1_setup_fifo_mode(enum fxls8471qr1_fifo_mode fifo_mode){
	local_fifo_config &= 0x3F;
	switch(fifo_mode)
	{
		case FXLS8471QR1_FIFO_OFF
			local_fifo_config |= F_SETUP_FIFO_OFF;
			break;
		case FXLS8471QR1_FIFO_CIRCULAR_BUFFER
			local_fifo_config |= F_SETUP_FIFO_CIRCULAR_BUFFER;
			break;
		case FXLS8471QR1_FIFO_STOP_ACCEPTING
			local_fifo_config |= F_SETUP_FIFO_STOP_ACCEPTING;
			break;
		case FXLS8471QR1_FIFO_TRIGGER
			local_fifo_config |= F_SETUP_FIFO_TRIGGER;
			break;
	}
	fxls8471qr1_write_register(REG_F_SETUP, local_fifo_config);
}

void fxls8471qr1_setup_fifo_watermark(uint8_t watermark){
	watermark &= 0x3F;
	local_fifo_config &= 0xC0;
	local_fifo_config |= watermark;
	fxls8471qr1_write_register(REG_F_SETUP, local_fifo_config);
}

void fxls8471qr1_setup_xyz_fs_range(enum fxls8471qr1_fs_range fs_range){
	local_xyz_data_config &= 0x10;
	switch(fs_range){
		case FXLS8471QR1_FS_RANGE_244
			local_xyz_data_config |= FS_RANGE_244;
			break;
		case FXLS8471QR1_FS_RANGE_488
			local_xyz_data_config |= FS_RANGE_488;
			break;
		case FXLS8471QR1_FS_RANGE_976
			local_xyz_data_config |= FS_RANGE_976;
			break;
	}
	fxls8471qr1_write_register(REG_XYZ_DATA_CFG, local_xyz_data_config);
}

void fxls8471qr1_setup_xyz_hp(bool hp_on){
	local_xyz_data_config &= 0x0F;
	if(hp_on)
		local_xyz_data_config |= FIFTH_BIT_ON;
	fxls8471qr1_write_register(REG_XYZ_DATA_CFG, local_xyz_data_config);
}

void fxls8471qr1_setup_ctrl_reg1_dr(enum fxls8471qr1_ctrl_reg1_dr data_rate){
	local_ctrl_reg1_config &= 0xC7;
	switch(data_rate){
		case FXLS8471QR1_DR_800_HZ
			local_ctrl_reg1_config |= DR_800_HZ;
			break;
		case FXLS8471QR1_DR_400_HZ
			local_ctrl_reg1_config |= DR_400_HZ;
			break;
		case FXLS8471QR1_DR_200_HZ
			local_ctrl_reg1_config |= DR_200_HZ;
			break;
		case FXLS8471QR1_DR_100_HZ
			local_ctrl_reg1_config |= DR_100_HZ;
			break;
		case FXLS8471QR1_DR_50_HZ
			local_ctrl_reg1_config |= DR_50_HZ;
			break;
		case FXLS8471QR1_DR_12_5_HZ
			local_ctrl_reg1_config |= DR_12_5_HZ;
			break;
		case FXLS8471QR1_DR_6_25_HZ
			local_ctrl_reg1_config |= DR_6_25_HZ;
			break;
		case FXLS8471QR1_DR_1_5625_HZ
			local_ctrl_reg1_config |= DR_1_5625_HZ;
			break;
	}
	fxls8471qr1_write_register(REG_CTRL_REG1, local_ctrl_reg1_config);
}

void fxls8471qr1_setup_ctrl_reg4_interrupt_enable(enum fxls8471qr1_ctrl_reg4_interrupt interrupt, bool enable){
	switch(interrupt){
		case FXLS8471QR1_INTERRUPT_ASLP
			local_ctrl_reg4_config &= ~EIGHTH_BIT_ON;
			if (enable)
				local_ctrl_reg4_config |= EIGHTH_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_FIFO
			local_ctrl_reg4_config &= ~SEVENTH_BIT_ON;
			if (enable)
				local_ctrl_reg4_config |= SEVENTH_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_TRANS
			local_ctrl_reg4_config &= ~SIXTH_BIT_ON;
			if (enable)
				local_ctrl_reg4_config |= SIXTH_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_LNDPRT
			local_ctrl_reg4_config &= ~FIFTH_BIT_ON;
			if (enable)
				local_ctrl_reg4_config |= FIFTH_BIT_ON;
;
			break;
		case FXLS8471QR1_INTERRUPT_PULSE
			local_ctrl_reg4_config &= ~FOURTH_BIT_ON;
			if (enable)
				local_ctrl_reg4_config |= FOURTH_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_FFMT
			local_ctrl_reg4_config &= ~THIRD_BIT_ON;
			if (enable)
				local_ctrl_reg4_config |= THIRD_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_A_VECM
			local_ctrl_reg4_config &= ~SECOND_BIT_ON;
			if (enable)
				local_ctrl_reg4_config |= SECOND_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_DRDY
			local_ctrl_reg4_config &= ~FIRST_BIT_ON;
			if (enable)
				local_ctrl_reg4_config |= ~FIRST_BIT_ON;
			break;
	};
	fxls8471qr1_write_register(REG_CTRL_REG4, local_ctrl_reg4_config);
}

//pin is true, INT1 pin
//pin is false, INT2 pin
void fxls8471qr1_setup_ctrl_reg5_interrupt_route_toggle(enum fxls8471qr1_ctrl_reg5_interrupt_route route, bool pin){
	switch(route){
		case FXLS8471QR1_INTERRUPT_RT_ASLP
			local_ctrl_reg5_config &= ~EIGHTH_BIT_ON;
			if (pin)
				local_ctrl_reg5_config |= EIGHTH_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_RT_FIFO
			local_ctrl_reg5_config &= ~SEVENTH_BIT_ON;
			if (pin)
				local_ctrl_reg5_config |= SEVENTH_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_RT_TRANS
			local_ctrl_reg5_config &= ~SIXTH_BIT_ON;
			if (pin)
				local_ctrl_reg5_config |= SIXTH_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_RT_LNDPRT
			local_ctrl_reg5_config &= ~FIFTH_BIT_ON;
			if (pin)
				local_ctrl_reg5_config |= FIFTH_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_RT_PULSE
			local_ctrl_reg5_config &= ~FOURTH_BIT_ON;
			if (pin)
				local_ctrl_reg5_config |= FOURTH_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_RT_FFMT
			local_ctrl_reg5_config &= ~THIRD_BIT_ON;
			if (pin)
				local_ctrl_reg5_config |= THIRD_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_RT_A_VECM
			local_ctrl_reg5_config &= ~SECOND_BIT_ON;
			if (pin)
				local_ctrl_reg5_config |= SECOND_BIT_ON;
			break;
		case FXLS8471QR1_INTERRUPT_RT_DRDY
			local_ctrl_reg5_config &= ~FIRST_BIT_ON;
			if (pin)
				local_ctrl_reg5_config |= FIRST_BIT_ON;
			break;
	};
	fxls8471qr1_write_register(REG_CTRL_REG5, local_ctrl_reg5_config);
}

void fxls8471qr1_record_latest_xyz_data(XYZ_DATA *xyzAccelData, int fifoReadNum){
	uint8_t Buffer[DATA_READ_LENGTH];
	for(int i = 0; i < fifoReadNum; i++){
		fxls8471qr1_read_packet(REG_OUT_X_MSB, Buffer, DATA_READ_LENGTH);
		xyzAccelData->x = ((Buffer[0] << 8) | Buffer[1]) >> 2;
		xyzAccelData->y = ((Buffer[2] << 8) | Buffer[3]) >> 2;
		xyzAccelData->z = ((Buffer[4] << 8) | Buffer[5]) >> 2;
	}
}
*/

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
      data.x = op_buffer[REG_OUT_X_MSB] << 8 | op_buffer[REG_OUT_X_LSB];
      data.y = op_buffer[REG_OUT_Y_MSB] << 8 | op_buffer[REG_OUT_Y_LSB];
      data.z = op_buffer[REG_OUT_Z_MSB] << 8 | op_buffer[REG_OUT_Z_LSB];
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
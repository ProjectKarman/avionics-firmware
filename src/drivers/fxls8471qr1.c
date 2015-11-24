/*
 * fxls8471qr1.c
 *
 * Created: 10/08/2015 4:56:25 PM
 *  Author: Brendan
 */ 

#include "asf.h"
#include "fxls8471qr1.h"
#include "usart_spi.h"
#include <util/delay.h>
#include <string.h>


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


// peripheral definitions
#define SPI_CTRL_USART USARTC1
#define SPI_CTRL (&SPI_CTRL_USART)

// function prototypes
static void spi_startframe(void);
static void spi_endframe(void);

// driver variables
static uint8_t local_fifo_config;

struct usart_spi_device conf = {
	.id = SPI_CS_PIN
};

void fxls8471qr1_init() {
	
	ioport_set_pin_dir(SPI_CS_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SPI_MISO_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SPI_MOSI_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(SPI_SCLK_PIN, IOPORT_DIR_OUTPUT);
	
	struct usart_spi_device conf = {
		.id = SPI_CS_PIN
	};

	usart_spi_init(SPI_CTRL);
	usart_spi_setup_device(SPI_CTRL, &conf, SPI_MODE_0, 1000000, 0);
	spi_endframe();
}

void spi_startframe(void){
	ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_LOW);
}

void spi_endframe(void){
	ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_HIGH);
}

void fxls8471qr1_write_register(uint8_t address, uint8_t new_value){
	spi_startframe();
	usart_spi_write_single(SPI_CTRL, SPICMD_W_REGISTER_MSB(address));
	usart_spi_write_single(SPI_CTRL, SPICMD_W_REGISTER_LSB(address));
	usart_spi_write_single(SPI_CTRL, new_value);
	spi_endframe();
}

void fxls8471qr1_read_register(uint8_t address, uint8_t *reg_value){
	spi_startframe();
	usart_spi_write_single(SPI_CTRL, SPICMD_R_REGISTER_MSB(address));
	usart_spi_write_single(SPI_CTRL, SPICMD_R_REGISTER_LSB(address));
	usart_spi_read_single(SPI_CTRL, reg_value);
	spi_endframe();
}

//void fxls8471qr1_setup_fifo(enum fxls8471qr1_fifo_mode fifo_mode, uint8_t watermark){
	//switch(fifo_mode)
	//{
		//case FXLS8471QR1_FIFO_OFF:
			//
	//}
	//fxls8471_write_register()
//}

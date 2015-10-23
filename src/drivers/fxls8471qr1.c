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


// peripheral definitions
#define SPI_CTRL_USART USARTC1
#define SPI_CTRL (&SPI_CTRL_USART)

// function prototypes
static void spi_startframe(void);
static void spi_endframe(void);

// driver variables
static uint8_t local_fifo_config;
static uint8_t local_xyz_data_config;
static uint8_t local_ctrl_reg1_config;
static uint8_t local_ctrl_reg4_config;
static uint8_t local_ctrl_reg5_config;

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

void fxls8471qr1_read_packet(uint8_t address, uint8_t *reg_value, int len){
	spi_startframe();
	usart_spi_write_single(SPI_CTRL, SPICMD_R_REGISTER_MSB(address));
	usart_spi_write_single(SPI_CTRL, SPICMD_R_REGISTER_LSB(address));
	usart_spi_read_packet(SPI_CTRL, reg_value, len);
	spi_endframe();
}

void fxls8471qr1_setup_fifo_mode(enum fxls8471qr1_fifo_mode fifo_mode){
	local_fifo_config &= 0x3F;
	switch(fifo_mode)
	{
		case FXLS8471QR1_FIFO_OFF:
			local_fifo_config |= F_SETUP_FIFO_OFF;
			break;
		case FXLS8471QR1_FIFO_CIRCULAR_BUFFER:
			local_fifo_config |= F_SETUP_FIFO_CIRCULAR_BUFFER;
			break;
		case FXLS8471QR1_FIFO_STOP_ACCEPTING:
			local_fifo_config |= F_SETUP_FIFO_STOP_ACCEPTING;
			break;
		case FXLS8471QR1_FIFO_TRIGGER:
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
		case FXLS8471QR1_FS_RANGE_244:
			break;
		case FXLS8471QR1_FS_RANGE_488:
			local_xyz_data_config |= 0x01;
			break;
		case FXLS8471QR1_FS_RANGE_976:
			local_xyz_data_config |= 0x02;
			break;
	}
	fxls8471qr1_write_register(REG_XYZ_DATA_CFG, local_xyz_data_config);
}

void fxls8471qr1_setup_xyz_hp(bool hp_on){
	local_xyz_data_config &= 0x0F;
	if(hp_on)
		local_xyz_data_config |= 0x10;
	fxls8471qr1_write_register(REG_XYZ_DATA_CFG, local_xyz_data_config);
}

void fxls8471qr1_setup_ctrl_reg1_dr(enum fxls8471qr1_ctrl_reg1_dr data_rate){
	local_ctrl_reg1_config &= 0xC7;
	switch(data_rate){
		case FXLS8471QR1_DR_800_HZ:
			break;
		case FXLS8471QR1_DR_400_HZ:
			local_ctrl_reg1_config |= 0x08;
			break;
		case FXLS8471QR1_DR_200_HZ:
			local_ctrl_reg1_config |= 0x10;
			break;
		case FXLS8471QR1_DR_100_HZ:
			local_ctrl_reg1_config |= 0x18;
			break;
		case FXLS8471QR1_DR_50_HZ:
			local_ctrl_reg1_config |= 0x20;
			break;
		case FXLS8471QR1_DR_12_5_HZ:
			local_ctrl_reg1_config |= 0x28;
		case FXLS8471QR1_DR_6_25_HZ:
			local_ctrl_reg1_config |= 0x30;
			break;
		case FXLS8471QR1_DR_1_5625_HZ:
			local_ctrl_reg1_config |= 0x38;
			break;
	}
	fxls8471qr1_write_register(REG_CTRL_REG1, local_ctrl_reg1_config);
}

void fxls8471qr1_setup_ctrl_reg4_interrupt_enable(enum fxls8471qr1_ctrl_reg4_interrupt interrupt, bool enable){
	switch(interrupt){
		case FXLS8471QR1_INTERRUPT_ASLP:
			local_ctrl_reg4_config &= 0x7F;
			if (enable)
				local_ctrl_reg4_config |= ~0x7F;
			break;
		case FXLS8471QR1_INTERRUPT_FIFO:
			local_ctrl_reg4_config &= 0xBF;
			if (enable)
				local_ctrl_reg4_config |= ~0xBF;
			break;
		case FXLS8471QR1_INTERRUPT_TRANS:
			local_ctrl_reg4_config &= 0xDF;
			if (enable)
				local_ctrl_reg4_config |= ~0xDF;
			break;
		case FXLS8471QR1_INTERRUPT_LNDPRT:
			local_ctrl_reg4_config &= 0xEF;
			if (enable)
				local_ctrl_reg4_config |= ~0xEF;
			break;
		case FXLS8471QR1_INTERRUPT_PULSE:
			local_ctrl_reg4_config &= 0xF7;
			if (enable)
				local_ctrl_reg4_config |= ~0xF7;
			break;
		case FXLS8471QR1_INTERRUPT_FFMT:
			local_ctrl_reg4_config &= 0xFB;
			if (enable)
				local_ctrl_reg4_config |= ~0xFB;
			break;
		case FXLS8471QR1_INTERRUPT_A_VECM:
			local_ctrl_reg4_config &= 0xFD;
			if (enable)
				local_ctrl_reg4_config |= ~0xFD;
			break;
		case FXLS8471QR1_INTERRUPT_DRDY:
			local_ctrl_reg4_config &= 0xFE;
			if (enable)
				local_ctrl_reg4_config |= ~0xFE;
			break;
	};
	fxls8471qr1_write_register(REG_CTRL_REG4, local_ctrl_reg4_config);
}

//pin is true, INT1 pin
//pin is false, INT2 pin
void fxls8471qr1_setup_ctrl_reg5_interrupt_route_toggle(enum fxls8471qr1_ctrl_reg5_interrupt_route route, bool pin){
	switch(route){
		case FXLS8471QR1_INTERRUPT_RT_ASLP:
			local_ctrl_reg5_config &= 0x7F;
			if (pin)
				local_ctrl_reg5_config |= ~0x7F;
			break;
		case FXLS8471QR1_INTERRUPT_RT_FIFO:
			local_ctrl_reg5_config &= 0xBF;
			if (pin)
				local_ctrl_reg5_config |= ~0xBF;
			break;
		case FXLS8471QR1_INTERRUPT_RT_TRANS:
			local_ctrl_reg5_config &= 0xDF;
			if (pin)
				local_ctrl_reg5_config |= ~0xDF;
			break;
		case FXLS8471QR1_INTERRUPT_RT_LNDPRT:
			local_ctrl_reg5_config &= 0xEF;
			if (pin)
				local_ctrl_reg5_config |= ~0xEF;
			break;
		case FXLS8471QR1_INTERRUPT_RT_PULSE:
			local_ctrl_reg5_config &= 0xF7;
			if (pin)
				local_ctrl_reg5_config |= ~0xF7;
			break;
		case FXLS8471QR1_INTERRUPT_RT_FFMT:
			local_ctrl_reg5_config &= 0xFB;
			if (pin)
				local_ctrl_reg5_config |= ~0xFB;
			break;
		case FXLS8471QR1_INTERRUPT_RT_A_VECM:
			local_ctrl_reg5_config &= 0xFD;
			if (pin)
				local_ctrl_reg5_config |= ~0xFD;
			break;
		case FXLS8471QR1_INTERRUPT_RT_DRDY:
			local_ctrl_reg5_config &= 0xFE;
			if (pin)
				local_ctrl_reg5_config |= ~0xFE;
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
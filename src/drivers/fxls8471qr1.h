/*
 * fxls8471qr1.h
 *
 * Created: 10/08/2015 4:54:05 PM
 *  Author: Brendan
 */ 


#ifndef FXLS8471QR1_H_
#define FXLS8471QR1_H_


#include <stdint.h>
#include <string.h>

enum fxls8471qr1_fifo_mode {
	FXLS8471QR1_FIFO_OFF,
	FXLS8471QR1_FIFO_CIRCULAR_BUFFER,
	FXLS8471QR1_FIFO_STOP_ACCEPTING,
	FXLS8471QR1_FIFO_TRIGGER,
};

void fxls8471qr1_init(void);

void fxls8471qr1_write_register(uint8_t address, uint8_t new_value);
void fxls8471qr1_read_register(uint8_t address, uint8_t *reg_value);
//void fxls8471qr1_setup_fifo(uint8_t watermark);
 
 
#endif /* FXLS8471QR1_H_ */
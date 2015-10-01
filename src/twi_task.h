/*
 * twi_task.h
 *
 * Created: 3.9.2014 0:11:16
 * Derived from the AVR1308 Example
 *  Author: klaxalk
 */ 

#include "avr/io.h"

#ifndef TWI_TASK_H_
#define TWI_TASK_H_

// the USART for testing the loopback
#define PC_USART2	USARTD0

// Defining an example slave address
#define SLAVE_ADDRESS	0x55

// BAUDRATE 100kHz and Baudrate Register Settings
#define BAUDRATE	100000
#define TWI_BAUDSETTING TWI_BAUD(F_CPU, BAUDRATE)

void TWIE_SlaveProcessData(void);
void twi_example(void *p);


#endif /* TWI_TASK_H_ */
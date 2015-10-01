/*
 * twi_task.c
 *
 * Created: 3.9.2014 0:05:36
 * Derived from the AVR1308 Example
 *  Author: klaxalk
 */ 

#include "twi_task.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
#include "usart_driver_RTOS.h"

// Global variables
TWI_Master_t twiMaster;	// TWI master module
TWI_Slave_t twiSlave;	// TWI slave module

// this function processes received data on the I2C Slave line
// It is call by the I2C driver
void TWIE_SlaveProcessData(void) {
	
	// takes the received character, increments it's value and sends it back
	twiSlave.sendData[0] = twiSlave.receivedData[0]+1;
}

// TWIC Master Interrupt vector
ISR(TWIC_TWIM_vect)
{
	TWI_MasterInterruptHandler(&twiMaster);
}

// TWIE Slave Interrupt vector
ISR(TWIE_TWIS_vect)
{
	TWI_SlaveInterruptHandler(&twiSlave);
}

/* This task shows the example of TWI (I2C) master and slave. There are two TWI interfaces set up, one as the
master, one as the slave. They are both connected to each other. The task receives characters from the USART,
sends it to the TWI slave. The slave takes the characters, increments their value and returns them to the master.
Then they are returned through the USART. */
void twi_example(void *p) {
	
	taskDISABLE_INTERRUPTS();
	
	// Initialize TWI master on PORTC
	TWI_MasterInit(&twiMaster, &TWIC, TWI_MASTER_INTLVL_LO_gc, TWI_BAUDSETTING);

	// Initialize TWI slave on PORTE
	TWI_SlaveInitializeDriver(&twiSlave, &TWIE, TWIE_SlaveProcessData);
	TWI_SlaveInitializeModule(&twiSlave, SLAVE_ADDRESS, TWI_SLAVE_INTLVL_LO_gc);

	// Enable LO interrupt level (needed without the FreeRTOS)
	// PMIC.CTRL |= PMIC_LOLVLEN_bm;

	// turn on USART on port
	char inChar;
	UsartBuffer * pc_usart_buffer = usartBufferInitialize(&PC_USART2, BAUD19200, 128);
	
	taskENABLE_INTERRUPTS();

	while (1) {
		
		// wait for some characters in USART input buffer
		while (!usartBufferGetByte(pc_usart_buffer, &inChar, 0)) {}
		
		// send the character to I2C Slave
		TWI_MasterWriteRead(&twiMaster, SLAVE_ADDRESS, &inChar, 1, 1);
		
		// Wait until transaction is complete
		while (twiMaster.status != TWIM_STATUS_READY) {}
		
		// send the received character back to the USART
		usartBufferPutByte(pc_usart_buffer, twiMaster.readData[0], 10);
	}
}
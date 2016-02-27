/*
 * hmc5883l.c
 *
 * Created: 2/11/2016 9:52:19 PM
 *  Author: Andrew Kaster
 *
 *
 *	A bunch of code (c) Atmel Corporation used as basis for this file (esp ISR stuff)
 */ 

 #include "hmc5883l.h"
 #include <stdbool.h>
 #include <stdint.h>
 #include <string.h>

 #include "asf.h"
 #include "FreeRTOS.h"
 #include "queue.h"
 #include "semphr.h"
 #include "task.h"

#define HMC5883L_CRA_DEFAULT 0x10
#define HMC5883L_CRB_DEFAULT 0x20	//First three bits determine sensor gain
									//Value of 001 is resolution of 0.92mG / count
#define HMC5883L_BUFFER_SIZE 6

#define HMC5883L_ADDRESS_BASE 0x7A
//#define HMC5883L_WRITE_ADDRESS 0x3C //ADDRESS_BASE << 1
//#define HMC5883L_READ_ADDRESS 0x3D  //ADDRESS_BASE << 1 | 0x01


// TWI Peripheral Config (copied from ms5607_02ba.c May need tweaking)
#define TWI_MASTER       TWIE
#define TWI_MASTER_PORT  PORTE
#define TWI_FREQ        400000
#define TWI_BAUD_REG 35 // ((F_SYS / (2 * TWI_FREQ)) - 5)

enum hmc5883l_command_type {
	CMD_TYPE_WRITE_ASYNC,
	CMD_TYPE_REQ_READ,
	CMD_TYPE_READ_ASYNC
};

#ifndef lambda
#define lambda(l_ret_type, l_arguments, l_body)         \
({                                                    \
  l_ret_type l_anonymous_functions_name l_arguments   \
  l_body                                            \
  &l_anonymous_functions_name;                        \
})
#endif

//local variables
static hmc5883l_rawdata_t rawdata;
static enum hmc5883l_sensor_mode_t sensor_mode;
static hmc5883l_angle_t curr_angles;
static enum hmc5883l_command_type current_command_type;
static uint8_t op_buffer[HMC5883L_BUFFER_SIZE];
static uint8_t op_buffer_len;
volatile static uint8_t op_buffer_index;
static hmc5883l_callback_t current_op_callback;
static Status_bool_t sent_req_bytes;

//local functions
static void write_command_async(uint16_t cmd, hmc5883l_callback_t callback);
static void read_command_async(uint8_t read_len, hmc5883l_callback_t callback);
//static void write_handler();
//static void read_handler();

//Initialization function
void hmc5883l_init(void){
	//TWI magic
	TWI_MASTER_PORT.PIN0CTRL |= PORT_OPC_WIREDANDPULL_gc;
	TWI_MASTER_PORT.PIN1CTRL |= PORT_OPC_WIREDANDPULL_gc;
	
	PR.PRPE &= ~PR_TWI_bm; // Enabled TWI module clock
	
	TWI_MASTER.MASTER.BAUD = TWI_BAUD_REG;
	//Set parameters for CTRLA and CTRLB registers on device
	TWI_MASTER.MASTER.CTRLA |= TWI_MASTER_ENABLE_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_INTLVL_MED_gc;
	TWI_MASTER.MASTER.CTRLB |= TWI_MASTER_QCEN_bm | TWI_MASTER_SMEN_bm;
	TWI_MASTER.MASTER.STATUS |= TWI_MASTER_BUSSTATE_IDLE_gc;


	//Set up task level structures

	//Set options for configuration registers
	write_command_async((HMC5883L_CRA << 8) & HMC5883L_CRA_DEFAULT, lambda(void, (void),{
    write_command_async((HMC5883L_CRB << 8) & HMC5883L_CRB_DEFAULT, NULL); })); //First set CRA to the default, then CRB to the default
}

/**Send a write command asynchronously. i.e. tell the TWI hardware we want to write something
	Then wait for the interrupt to trigger. Data is sent in the ISR
 */
void write_command_async(uint16_t cmd, hmc5883l_callback_t callback) {
	//Our command can be up to two bytes long. We store each byte in our buffer separately
	*op_buffer = cmd & 0xFF;
	*(op_buffer + 1) = (cmd >> 8);

	//Set global variables for use in ISR
	op_buffer_index = 0;
	if(op_buffer[1] == 0)	//If we have a one byte command, our buffer length is one
		op_buffer_len = 1;
	else					//Two byte command --> buffer length is 2
		op_buffer_len = 2;
	current_command_type = CMD_TYPE_WRITE_ASYNC;
	current_op_callback = callback;
	//Inform the master that we want to write something to this address
	TWI_MASTER.MASTER.ADDR = HMC5883L_ADDRESS_BASE << 1;
}

/**Send a read command asynchronously. i.e. tell the TWI hardware that we want to read something
	Then wait for the interrupt to trigger. Data is stored into the op_buffer in the ISR
*/
void read_command_async(uint8_t read_len, hmc5883l_callback_t callback) {
	//First we send how many bytes we want to read
  *op_buffer = read_len; //Store our requested bytes into the buffer
  op_buffer_len = 1;
  op_buffer_index = 0;
	current_op_callback = callback;
	current_command_type = CMD_TYPE_REQ_READ; //We're going to request a read
  sent_req_bytes = 0; //we haven't told the slave how many bytes we want
	TWI_MASTER.MASTER.ADDR = HMC5883L_ADDRESS_BASE << 1 | 0x01; //Tell slave to send us data
}

/** Read data from the compass once, and store its info in our raw data pointer 
*/
void hmc5883l_read_data_single (hmc5883l_rawdata_t* raw_data_ptr)
{
	//Should totes grab a "command running" semaphore here?
	write_command_async((HMC5883L_MODE << 8) & HMC5883L_MODE_SINGLE, lambda(void, (void), {
    read_command_async(HMC5883L_BUFFER_SIZE, NULL);}));     //Send "Single measurement mode" message, then Read six bytes


  //Definitely need to check to make sure our data transfer is complete here
  //while loop based on a volatile variable? semaphore status? check TWI_MASTER.MASTER.STATUS?
  //We could use a data queue and have the ISR put stuff in it?
  
	//Store our data in our variable
	raw_data_ptr->x_magnitude = (op_buffer[0] << 8) | op_buffer[1];
	raw_data_ptr->y_magnitude = (op_buffer[2] << 8) | op_buffer[3];
	raw_data_ptr->z_magnitude = (op_buffer[4] << 8) | op_buffer[5];

	//We should verify this data using the DATA_MAX, DATA_MIN, and DATA_ERROR #defines
}

/**TWI Master interrupt service routine
	shamelessly copied from Atmel Corporation
	also modified for our needs because who cares about errors?
	used as template for one below
*/
/*
ISR(TWIE_TWIM_vect)
{
	//Start by checking the bus status and making decisions based on what the status is
	uint8_t const status = TWI_MASTER.MASTER.STATUS;

	//We uh... lost arbitration? whatever that means
	if( status & TWI_MASTER_ARBLOST_bm)
	{
		// reinforce that we indeed lost arbitration..?
		TWI_MASTER.MASTER.STATUS = status | TWI_MASTER_ARBLOST_bm;
		//Tell the master to stop
		TWI_MASTER.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		//We're busy. We should def tell somebody about that
	}
	//We have a bus error or a NACK
	else if((status & TWI_MASTER_BUSERR_bm) ||(status & TWI_MASTER_RXACK_bm))
	{
		//stop sending stuff
		TWI_MASTER.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		//We should definitely tell somebody that we had an IO error
	}
	//We're trying to write something
	else if(status & TWI_MASTER_WIF_bm)
	{
		//call that nifty write handler
		hmc5883l_write_handler();
	}
	else if (status & TWI_MASTER_RIF_bm)
	{
		//call that nifty read handler
		hmc5883l_read_handler();
	}
	else
	{
		//something is seriously wrong if we get here
	}
}

// Do stuff when we get an interrupt that says we're ready to write stuff
void hmc5883l_write_handler()
{

}

// Do stuff when we get an interrupt that says we're ready to read stuff
void hmc5883l_read_handler()
{

}
 */


/*This one is based on Nigel's odd code from the altimeter driver and Atmel's code
	Basic idea is we control the bus, screw checking the status, use a state machine instead.
*/
ISR(TWIE_TWIM_vect)
{
	switch(current_command_type)
	{
  case CMD_TYPE_REQ_READ: //We're telling the slave to get ready for a read
      if(sent_req_bytes == 0) //We haven't already told the slave to get ready
      {
        TWI_MASTER.MASTER.DATA = op_buffer[op_buffer_index]; //We wrote how many bytes we want into the op_buffer, so send them out on the line
        op_buffer_len = op_buffer[op_buffer_index];  //*op_buffer is our buffer length for the read, so let's make that our buffer_len
        op_buffer_index = 0; //make sure buffer index is 0
        sent_req_bytes = 1; //We just sent how many bytes we want
      }
      else //sent_req_bytes is 1, so we just sent how many bytes we want
      {
        TWI_MASTER.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;  //send a stop
        current_command_type = CMD_TYPE_READ_ASYNC; //Set the state to read_async so we'll react properly when the slave sends us data
      }
    break;
	case CMD_TYPE_READ_ASYNC: //We've initiated a read command
		if(op_buffer_index < op_buffer_len) // We're expecting more data to come in
		{
			op_buffer[op_buffer_index++] =TWI_MASTER.MASTER.DATA; //Put the data in the buffer

			if(op_buffer_index < op_buffer_len) // There's still more, so send an ACK and read again
			{
				TWI_MASTER.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;
			}
			else //We're done reading data, so send a NACK and STOP
			{
				TWI_MASTER.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;
				//We could totes tell someone that the transfer status went well here
				if(current_op_callback)
        {
          current_op_callback(); //If we had a function call queued up, let's do it now
        }
			}
		}
		else //Either we weren't expecting data, or we got too much data
		{
			TWI_MASTER.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
			//That's also something we could tell someone about (yikes)
		}
		break;
	case CMD_TYPE_WRITE_ASYNC: //We're initiating a write command
		//Only problem here is if we think we're supposed to be reading but the slave thinks we're trying to read, we can't know
		if(op_buffer_index < op_buffer_len) //we still have data to write
		{
			TWI_MASTER.MASTER.DATA = op_buffer[op_buffer_index++]; //queue the next byte up for transfer
		}
		else //We're done here, send STOP
		{
			TWI_MASTER.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
			//We could totes tell someone that the transfer went well here
			if(current_op_callback)
			{
  			current_op_callback(); //If we had a function call queued up, let's do it now
			}
		}
		break;
	default: //We should never get here
		//Debug oh god why variable += 1
		break;
	}
}


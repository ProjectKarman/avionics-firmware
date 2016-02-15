/*
 * hmc5883l.c
 *
 * Created: 2/11/2016 9:52:19 PM
 *  Author: Andrew Kaster
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
#define HMC5883L_MESSAGE_SIZE 6

// TWI Peripheral Config (copied from ms5607_02ba.c May need tweaking)
#define TWI_MASTER       TWIE
#define TWI_MASTER_PORT  PORTE
#define TWI_FREQ        400000
#define TWI_BAUD_REG 35 // ((F_SYS / (2 * TWI_FREQ)) - 5)

//local variables
static char cmd_buffer[HMC5883L_MESSAGE_SIZE];
static char cmd_size = HMC5883L_MESSAGE_SIZE;
static hmc5883l_rawdata_t rawdata;
static hmc5883l_reg_t registers;
static hmc5883l_sensor_mode_t sensor_mode;
static hmc5883l_angle_t curr_angles;


//local functions
static void hmc5883l_send_i2c_message(char * buffer, char size);

//Initialization function
void hmc5883l_init(void){
	//TWI magic. No clue what this does. copied from ms5607_02ba.c May need tweaking
	TWI_MASTER_PORT.PIN0CTRL |= PORT_OPC_WIREDANDPULL_gc;
	TWI_MASTER_PORT.PIN1CTRL |= PORT_OPC_WIREDANDPULL_gc;
	
	PR.PRPE &= ~PR_TWI_bm; // Enabled TWI module clock
	
	TWI_MASTER.MASTER.BAUD = TWI_BAUD_REG;
	TWI_MASTER.MASTER.CTRLA |= TWI_MASTER_ENABLE_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_INTLVL_MED_gc;
	TWI_MASTER.MASTER.CTRLB |= TWI_MASTER_QCEN_bm | TWI_MASTER_SMEN_bm;
	TWI_MASTER.MASTER.STATUS |= TWI_MASTER_BUSSTATE_IDLE_gc;


}
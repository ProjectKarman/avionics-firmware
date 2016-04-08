/*
 * neutwi.h
 *
 *Implementation of a TWI Driver specific for ATxmega128A1U for Northeaster University
 *
 *Driver functions take a twi_data struct as input, and output data to a buffer included in this struct
 *
 *Interrupt Service Routine does not care what "state" your driver is in, but checks the master status for what to do
 *
 * Created: 3/3/2016 4:44:38 PM
 *  Author: Andrew Kaster
 */ 


#ifndef NEUTWI_H_
#define NEUTWI_H_

void twi_iterrupt_handler();








#endif /* NEUTWI_H_ */
/*
 * hmc5883l.c
 *
 * Created: 4/7/2016 8:49:58 PM
 *  Author: Andrew Kaster
 */ 


#include "hmc5883l.h"
 #include "asf.h"
 #include "FreeRTOS.h"
 #include "queue.h"
 #include "task.h"


#define HMC5883L_CRA_DEFAULT 0x10
#define HMC5883L_CRB_DEFAULT 0x20	//First three most significant bits determine sensor gain
//Value of 001 is resolution of 0.92mG / count
#define HMC5883L_DATA_SIZE 6

#define HMC5883L_ADDRESS_BASE 0x7A
//#define HMC5883L_WRITE_ADDRESS 0x3C //ADDRESS_BASE << 1
//#define HMC5883L_READ_ADDRESS 0x3D  //ADDRESS_BASE << 1 | 0x01

//Min and max values stored in data registers
#define HMC5883L_OUTPUT_MIN		 0xF800
#define HMC5883L_OUTPUT_MAX		 0x07FF

//In the event of ADC over/underflow or math overflow during bias calc
//Data register will have the value of -4096 until next valid measurement
#define HMC5883L_OUTPUT_ERROR	 0xF000


enum hmc5883l_sensor_mode_t{
  HMC5883L_MODE_CONTINUOUS,
  HMC5883L_MODE_SINGLE,
  HMC5883L_MODE_IDLE
};

enum hmc5883l_reg_t {
  HMC5883L_CRA,		//Control register A
  HMC5883L_CRB,		//Control register B
  HMC5883L_MODE,
  HMC5883L_XMSB,
  HMC5883L_XLSB,
  HMC5883L_ZMSB,
  HMC5883L_ZLSB,
  HMC5883L_YMSB,
  HMC5883L_YLSB,
  HMC5883L_STATUS,
  HMC5883L_IDA,		//Identification register A
  HMC5883L_IDB,		//Identification register B
  HMC5883L_IDC		//Identification register C
};

hmc5883l_t hmc5883l_obj;


void hmc5883l_init(){
  hmc5883l_obj.hmc5883l_data_queue = xQueueCreate(HMC5883L_DATA_SIZE, sizeof(uint8_t));

  /*For all write commands, first byte sent out is device address, followed by register number
  * and third byte is data
  * for read, first byte is device address, followed by register number and optional number
  * of bytes to read. If no bytes to read are sent, reg1ister pointer is simply set to
  * register number. After a register is read, register pointer will automatically increment
  * and send value in next register if not all requested bytes have been sent */

  //Set configuration registers
  //The data will need to change in configure function
  hmc5883l_obj.setConfigA.device_addr = HMC5883L_ADDRESS_BASE << 1;
  hmc5883l_obj.setConfigA.mode = TWI_WRITE_MODE;
  hmc5883l_obj.setConfigA.write_data[0] = HMC5883L_CRA;
  hmc5883l_obj.setConfigA.write_data[1] = HMC5883L_CRA_DEFAULT;
  hmc5883l_obj.setConfigA.length = 2;

  hmc5883l_obj.setConfigB.device_addr = HMC5883L_ADDRESS_BASE << 1;
  hmc5883l_obj.setConfigB.mode = TWI_WRITE_MODE;
  hmc5883l_obj.setConfigB.write_data[0] = HMC5883L_CRB;
  hmc5883l_obj.setConfigB.write_data[1] = HMC5883L_CRB_DEFAULT;
  hmc5883l_obj.setConfigB.length = 2;

  //Set measurement mode register
  //Default is continuous measurement mode. Must wait 6 ms before taking first measurement after
  //Sending this command
  hmc5883l_obj.setMode.device_addr = HMC5883L_ADDRESS_BASE << 1;
  hmc5883l_obj.setMode.mode = TWI_WRITE_MODE;
  hmc5883l_obj.setMode.write_data[0] = HMC5883L_MODE;
  hmc5883l_obj.setMode.write_data[1] = HMC5883L_MODE_CONTINUOUS; //Continuous measurement
  hmc5883l_obj.setMode.length = 2;

  //Read all six bytes of data at once
  hmc5883l_obj.getRawData.return_queue = hmc5883l_obj.hmc5883l_data_queue;
  hmc5883l_obj.getRawData.device_addr = HMC5883L_ADDRESS_BASE << 1 | 0x1;
  hmc5883l_obj.getRawData.mode = TWI_READ_MODE;
  hmc5883l_obj.getRawData.length = 6;

  //Set register pointer to register 3 for reset
  hmc5883l_obj.resetRegPointer.device_addr = HMC5883L_ADDRESS_BASE << 1;
  hmc5883l_obj.resetRegPointer.mode = TWI_WRITE_MODE;
  hmc5883l_obj.resetRegPointer.write_data[0] = HMC5883L_XMSB; //First data register
  hmc5883l_obj.resetRegPointer.length = 1;
}

//Send config messages
uint8_t hmc5883l_configure(){
  //Set the actual values we want to configuration and mode registers
  //Based on data sheet pages 12, 13 and 14
  uint8_t retVal = 0;

  hmc5883l_obj.setConfigA.write_data[1] = 0x70;
  hmc5883l_obj.setConfigB.write_data[1] = 0xA0;
  hmc5883l_obj.setMode.write_data[1] = HMC5883L_MODE_CONTINUOUS;

  retVal = twi_add_task_to_queue(&hmc5883l_obj.setConfigA);
  retVal &= twi_add_task_to_queue(&hmc5883l_obj.setConfigB);
  retVal &= twi_add_task_to_queue(&hmc5883l_obj.setMode);

  return retVal;
}

//Tell twi interface to grab the raw data from the magnetometer and store it in its queue
uint8_t hmc5883l_read_data(){
  return twi_add_task_to_queue(&hmc5883l_obj.getRawData);
}


//Read values from magnetometer and store values in raw data struct
//Time delay before calling this function depends on data rate set in register CRA
uint8_t hmc5883l_fetch_queue_data(hmc5883l_rawdata_t *rawdata_ptr){
  uint8_t status = 0;
  uint8_t queue_result[6];
  uint8_t queue_index = 0;

  status = xQueuePeek(hmc5883l_obj.getRawData.return_queue,queue_result, 0);

  if(status == 1){
    for(queue_index = 0; queue_index < 6; queue_index++){
      if(!xQueueReceive(hmc5883l_obj.getRawData.return_queue, &queue_result[queue_index], 0)){
        return 0;
      }
    }
    rawdata_ptr->x_magnitude = (queue_result[0] << 8);  //XMSB
    rawdata_ptr->x_magnitude |= queue_result[1];        //XMLB
    rawdata_ptr->z_magnitude = (queue_result[2] << 8);  //ZMSB
    rawdata_ptr->z_magnitude |= queue_result[3];        //ZLSB
    rawdata_ptr->y_magnitude = (queue_result[4] << 8);  //YMSB
    rawdata_ptr->y_magnitude |= queue_result[5];        //YLSB
  

    twi_add_task_to_queue(&hmc5883l_obj.resetRegPointer);
  }
  return status;
}


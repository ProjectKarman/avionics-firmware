/*
 * This is the Main C File for the avionics firmware
 */

#include "user_board.h"
#include "asf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "transceiver.h"
#include "sensor.h"

int main(void)
{	
  board_init();
  // ioport_set_pin_dir(LEDA, IOPORT_DIR_OUTPUT);
  // ioport_set_pin_dir(LEDB, IOPORT_DIR_OUTPUT);

  // start tasks
  
  // transceiver_start_task();
  sensor_start_task();

  vTaskStartScheduler();
  
  return 0;
}
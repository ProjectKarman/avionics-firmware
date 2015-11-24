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

#define LEDA IOPORT_CREATE_PIN(PORTA, 0)
#define LEDB IOPORT_CREATE_PIN(PORTA, 1)

void blink1(void *p);
void blink2(void *p);

void blink1(void *p) {	
	while (1) {
    ioport_toggle_pin_level(LEDA);
    vTaskDelay(1000);
	}
}

void blink2(void *p) {
	while (1) {
		ioport_toggle_pin_level(LEDB);
		vTaskDelay(100);
	}
}

int main(void)
{	
	board_init();

  ioport_set_pin_dir(LEDA, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(LEDB, IOPORT_DIR_OUTPUT);

	// start tasks
	xTaskCreate(blink1, "blink1", 64, NULL, 2, NULL);
	xTaskCreate(blink2, "blink2", 64, NULL, 2, NULL);
  // transceiver_start_task();
  
  sensor_start_task();

	vTaskStartScheduler();
	
	return 0;
}
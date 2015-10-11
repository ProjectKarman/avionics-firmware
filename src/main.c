/*
 * This is the Main C File for the avionics firmware
 */

#include "user_board.h"
#include "asf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ms5607_02ba.h"

void blink1(void *p) {	
	while (1) {
        PORTA.OUT ^= 0x01;
        vTaskDelay(1000);
	}
}

void blink2(void *p) {
	while (1) {
		PORTA.OUT ^= 0x02;
		vTaskDelay(100);
	}
}

int main(void)
{	
	PORTA.DIR |= 0x03;

	board_init();
	ms5607_02ba_init();
  
  
	// start tasks
	xTaskCreate(blink1, (signed char*) "blink1", 1024, NULL, 2, NULL);
	xTaskCreate(blink2, (signed char*) "blink2", 1024, NULL, 2, NULL);

	vTaskStartScheduler();
	
	return 0;
}
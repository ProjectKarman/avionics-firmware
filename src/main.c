/*
 * This is the Main C File for the avonics firmware
 */

#include "user_board.h"
#include "asf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// YELLOW LED on PORTA-0
#define	YELLOW	IOPORT_CREATE_PIN(PORTA, 0)

// RED LED on PORTA-4
#define	RED	IOPORT_CREATE_PIN(PORTA, 4)

void blink1(void *p) {
	
	while (1) {
		
		// ioport_toggle_pin_level(RED);
        PORTA.OUT ^= 0x40;
        vTaskDelay(1000);
	}
}

void blink2(void *p) {
	
	while (1) {
		PORTA.OUT ^= 0x80;
		//ioport_toggle_pin_level(YELLOW);
		vTaskDelay(100);
	}
}

int main(void)
{	
	PORTA.DIR |= 0xC0;

	board_init();
	
	// start tasks
	xTaskCreate(blink1, (signed char*) "blink1", 1024, NULL, 2, NULL);
	xTaskCreate(blink2, (signed char*) "blink2", 1024, NULL, 2, NULL);

	vTaskStartScheduler();
	
	return 0;
}
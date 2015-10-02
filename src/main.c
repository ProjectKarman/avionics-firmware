/*
 * main.c
 *
 * Created: 24.8.2014 15:10:04
 *  Author: klaxalk
 */ 


#include "user_board.h"
#include <avr/io.h>
#include <util/delay.h>
#include "sysclk.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ioport.h"

// YELLOW LED on PORTA-0
#define	YELLOW	IOPORT_CREATE_PIN(PORTA, 0)

// RED LED on PORTA-4
#define	RED	IOPORT_CREATE_PIN(PORTA, 4)

void blink1(void *p) {
	
	while (1) {
		
		// ioport_toggle_pin_level(RED);
        vTaskDelay(1000);
	}
}

void blink2(void *p) {
	
	while (1) {
		
		//ioport_toggle_pin_level(YELLOW);
		vTaskDelay(100);
	}
}

int main(void)
{	
		
	// prepare the i/o for LEDs
	//ioport_init();
	//ioport_set_pin_dir(RED, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_dir(YELLOW, IOPORT_DIR_OUTPUT);
	board_init();
	
	// clock init & enable system clock to all peripheral modules
	//sysclk_init();
	//sysclk_enable_module(SYSCLK_PORT_GEN, 0xff);
	//sysclk_enable_module(SYSCLK_PORT_A, 0xff);
	//sysclk_enable_module(SYSCLK_PORT_B, 0xff);
	//sysclk_enable_module(SYSCLK_PORT_C, 0xff);
	//sysclk_enable_module(SYSCLK_PORT_D, 0xff);
	//sysclk_enable_module(SYSCLK_PORT_E, 0xff);
	//sysclk_enable_module(SYSCLK_PORT_F, 0xff);
			
	// start tasks
	xTaskCreate(blink1, (signed char*) "blink1", 1024, NULL, 2, NULL);
	xTaskCreate(blink2, (signed char*) "blink2", 1024, NULL, 2, NULL);

	vTaskStartScheduler();
	
	return 0;
}
/*
 * This is the Main C File for the avionics firmware
 */

#include "user_board.h"
#include "asf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "nrf24l01p.h"

#define LEDA IOPORT_CREATE_PIN(PORTA, 0)
#define LEDB IOPORT_CREATE_PIN(PORTA, 1)
#define CE_PIN IOPORT_CREATE_PIN(PORTB, 7)

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

void data_test(void *p) {
  nrf24l01p_init();
  nrf24l01p_sync_regs();
  nrf24l01p_wake();
  _delay_ms(1.5);
  nrf24l01p_flush_rx_fifo();
  nrf24l01p_flush_tx_fifo();
  nrf24l01p_set_channel(20);
  nrf24l01p_set_data_rate(NRF24L01P_DR_2M);
  nrf24l01p_data_test();
}

int main(void)
{	
	board_init();
  ioport_set_pin_dir(LEDA, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(LEDB, IOPORT_DIR_OUTPUT);

	// start tasks
	xTaskCreate(blink1, (signed char*) "blink1", 1024, NULL, 2, NULL);
	xTaskCreate(blink2, (signed char*) "blink2", 1024, NULL, 2, NULL);
  xTaskCreate(data_test, (signed char*) "data", 1024, NULL, 2, NULL);

	vTaskStartScheduler();
	
	return 0;
}
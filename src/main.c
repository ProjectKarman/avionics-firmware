/*
 * This is the Main C File for the avionics firmware
 */

#include "user_board.h"
#include "asf.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "transceiver.h"

#define LEDA IOPORT_CREATE_PIN(PORTA, 0)
#define LEDB IOPORT_CREATE_PIN(PORTA, 1)

void blink1(void *p);
void blink2(void *p);
void gen_test_packets(void *p);

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

void gen_test_packets(void *p) {
  for(;;) {
    general_message_t *content = general_message_create();
    content->text = "Hello World!";
    content->len = strlen(content->text);

    transceiver_message_t *message = transceiver_message_create();
    message->type = TRANSCEIVER_MSG_TYPE_GENERAL;
    message->data = content;

    transceiver_send_message(message, 0);
    
    vTaskDelay(10);
  }
}

int main(void)
{	
	board_init();

  ioport_set_pin_dir(LEDA, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(LEDB, IOPORT_DIR_OUTPUT);

	// start tasks
	xTaskCreate(blink1, "blink1", 1024, NULL, 2, NULL);
	xTaskCreate(blink2, "blink2", 1024, NULL, 2, NULL);
  xTaskCreate(gen_test_packets, "test data", 1024, NULL, 2, NULL);
  transceiver_start_task();

	vTaskStartScheduler();
	
	return 0;
}
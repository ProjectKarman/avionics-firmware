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
    general_message_t *content1 = general_message_create();
    content1->text = "Hello World!";
    content1->len = strlen(content1->text);

    transceiver_message_t *message1 = transceiver_message_create();
    message1->type = TRANSCEIVER_MSG_TYPE_GENERAL;
    message1->data = content1;

    general_message_t *content2 = general_message_create();
    content2->text = "Hello World!";
    content2->len = strlen(content2->text);

    transceiver_message_t *message2 = transceiver_message_create();
    message2->type = TRANSCEIVER_MSG_TYPE_GENERAL;
    message2->data = content2;

    general_message_t *content3 = general_message_create();
    content3->text = "Hello World!";
    content3->len = strlen(content3->text);

    transceiver_message_t *message3 = transceiver_message_create();
    message3->type = TRANSCEIVER_MSG_TYPE_GENERAL;
    message3->data = content3;

    transceiver_send_message(message1, 0);
    transceiver_send_message(message2, 0);
    transceiver_send_message(message3, 0);
    
    vTaskDelay(3);
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
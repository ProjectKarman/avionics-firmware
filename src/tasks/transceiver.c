/*
 * transceiver.c
 *
 * Created: 10/8/2015 5:20:31 PM
 *  Author: Nigil Lee
 */ 

#include "asf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "transceiver.h"
#include "nrf24l01p.h"

#define FRAME_RATE 250
#define PACKET_BYTES 20

#define MESSAGE_QUEUE_DEPTH PACKET_BYTES

static void transceiver_task_loop(void *p);
static void init_nrf24l01p(void);
static void transmit_complete(enum dma_channel_status status);

QueueHandle_t transceiver_send_queue;
TaskHandle_t transceiver_task_handle;

void transceiver_start_task(void) {
  xTaskCreate(transceiver_task_loop, "transceiver", 200, NULL, 2, &transceiver_task_handle);
}

static void transceiver_task_loop(void *p) {
  init_nrf24l01p();

  // Setup queue
  transceiver_send_queue = xQueueCreate(MESSAGE_QUEUE_DEPTH, sizeof(struct transceiver_message *));

  // Make up data
  uint8_t test_data[32], i;

  for(i = 0; i < 32; i++) {
    test_data[i] = i;
  }

  uint8_t fifo_status;
  nrf24l01p_read_register(0x17, &fifo_status);

  nrf24l01p_init_tx_payload_xfer(test_data, 32, transmit_complete);
  vTaskSuspend(NULL);

  nrf24l01p_read_register(0x17, &fifo_status);

  for(;;) {
    //nrf24l01p_data_test();
    vTaskDelay(200);
  }
}

 static void init_nrf24l01p() {
  nrf24l01p_read_regs();
  nrf24l01p_wake();
  vTaskDelay(2);
  nrf24l01p_flush_rx_fifo();
  nrf24l01p_flush_tx_fifo();
  nrf24l01p_set_channel(20);
  nrf24l01p_set_data_rate(NRF24L01P_DR_2M);
  nrf24l01p_set_pa_power(NRF24L01P_PWR_N18DBM);
 }

 static void transmit_complete(enum dma_channel_status status) {
  xTaskResumeFromISR(transceiver_task_handle);
 }
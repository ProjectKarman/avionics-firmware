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

#define PROTOCOL_TIMER TCC1
#define PROTOCOL_FRAME_RATE 250

#define PACKET_BYTES 20

#define MESSAGE_QUEUE_DEPTH PACKET_BYTES

static void transceiver_task_loop(void *p);
static void init_nrf24l01p(void);
static void transmit_operation(void);
static void dma_xfer_complete_handler(void);
static void nrf24l01p_interrupt_handler(void);
static void protocol_timer_overflow_handler(void);

QueueHandle_t transceiver_send_queue;
TaskHandle_t transceiver_task_handle;

static volatile enum transeiver_status status;

void transceiver_start_task(void) {
  // Configure timer to generate frame phase interrupts
  tc_enable(&PROTOCOL_TIMER);
  tc_set_wgm(&PROTOCOL_TIMER, TC_WG_NORMAL);
  tc_write_clock_source(&PROTOCOL_TIMER, TC_CLKSEL_DIV4_gc);
  tc_write_period(&PROTOCOL_TIMER, F_CPU / (4 * PROTOCOL_FRAME_RATE));
  tc_set_overflow_interrupt_callback(&PROTOCOL_TIMER, protocol_timer_overflow_handler);
  tc_set_overflow_interrupt_level(&PROTOCOL_TIMER, TC_INT_LVL_MED); 

  xTaskCreate(transceiver_task_loop, "transceiver", 200, NULL, 2, &transceiver_task_handle);
}

static void transceiver_task_loop(void *p) {
  init_nrf24l01p();

  transceiver_send_queue = xQueueCreate(MESSAGE_QUEUE_DEPTH, sizeof(struct transceiver_message *));

  nrf24l01p_set_interrupt_mask(NRF24L01P_INTR_TX_DS);
  nrf24l01p_set_interrupt_pin_handler(nrf24l01p_interrupt_handler);



  /*
  // Make up data
  uint8_t test_data[32], i;
  for(i = 0; i < 32; i++) {
    test_data[i] = i;
  }

  uint8_t fifo_status;
  nrf24l01p_read_register(0x17, &fifo_status);

  //nrf24l01p_init_tx_payload_xfer(test_data, 32, transmit_complete);
  nrf24l01p_send_payload(test_data, 32);
  //vTaskSuspend(NULL);
  nrf24l01p_read_register(0x17, &fifo_status);
  //nrf24l01p_init_tx_payload_xfer(test_data, 32, transmit_complete);
  nrf24l01p_send_payload(test_data, 32);
  //vTaskSuspend(NULL);
  nrf24l01p_read_register(0x17, &fifo_status);
  //nrf24l01p_init_tx_payload_xfer(test_data, 20, transmit_complete);
  nrf24l01p_send_payload(test_data, 32);
  //vTaskSuspend(NULL);

  nrf24l01p_read_register(0x17, &fifo_status);
  */

  for(;;) {
    status = TRANSCEIVER_STATUS_IDLE;
    vTaskSuspend(NULL); // Suspend until we have to do something...

    switch(status) {
      case TRANSCEIVER_STATUS_TRANSMITTING:
        transmit_operation();
        break;
      case TRANSCEIVER_STATUS_RECEIVING:
        break;
      default:
        break;
    }
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

static void transmit_operation(void) {

 }

static void dma_xfer_complete_handler(void) {
  xTaskResumeFromISR(transceiver_task_handle);
 }

static void nrf24l01p_interrupt_handler(void) {

 }

static void protocol_timer_overflow_handler(void) {
  // We need to initiate a transmit frame
  status = TRANSCEIVER_STATUS_TRANSMITTING;
  xTaskResumeFromISR(transceiver_task_handle);
 }
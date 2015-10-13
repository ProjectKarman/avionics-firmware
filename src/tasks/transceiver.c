/*
 * transceiver.c
 *
 * Created: 10/8/2015 5:20:31 PM
 *  Author: Nigil Lee
 */ 

#include <stdbool.h>
#include "asf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "nrf24l01p.h"
#include "packet_history.h"
#include "message_types.h"
#include "transceiver.h"

#define PROTOCOL_TIMER TCC1
#define PROTOCOL_FRAME_RATE 250

#define PACKET_BYTES 20

#define MESSAGE_QUEUE_DEPTH PACKET_BYTES

enum transeiver_state {
  TRANSCEIVER_STATE_IDLE,
  TRANSCEIVER_STATE_SLEEP,
  TRANSCEIVER_STATE_TRANSMITTING,
  TRANSCEIVER_STATE_TX_PREP,
  TRANSCEIVER_STATE_RECEIVING,
};

static void transceiver_task_loop(void *p);
static void init_nrf24l01p(void);
static void tx_prep_operation(void);
static void transmit_operation(void);

static protocol_packet_t *general_message_to_packet(general_message_t *message);

static void dma_xfer_complete_handler(void);
static void nrf24l01p_interrupt_handler(void);
static void protocol_timer_overflow_handler(void);

TaskHandle_t transceiver_task_handle;
QueueHandle_t *transceiver_send_queue;

static volatile enum transeiver_state state;
static packet_history_t *packet_history;
static QueueHandle_t send_queue_a;
static QueueHandle_t send_queue_b;
static bool using_queue_a;

void transceiver_start_task(void) {
  xTaskCreate(transceiver_task_loop, "transceiver", 200, NULL, 2, &transceiver_task_handle);
  send_queue_a = xQueueCreate(MESSAGE_QUEUE_DEPTH, sizeof(struct transceiver_message *));
  send_queue_b = xQueueCreate(MESSAGE_QUEUE_DEPTH, sizeof(struct transceiver_message *));
  transceiver_send_queue = &send_queue_a;
  using_queue_a = true;

  packet_history = pvPortMalloc(sizeof(packet_history_t));
  packet_history_init(packet_history);

  // Configure timer to generate frame phase interrupts
  tc_enable(&PROTOCOL_TIMER);
  tc_set_wgm(&PROTOCOL_TIMER, TC_WG_NORMAL);
  tc_write_clock_source(&PROTOCOL_TIMER, TC_CLKSEL_DIV4_gc);
  tc_write_period(&PROTOCOL_TIMER, F_CPU / (4 * PROTOCOL_FRAME_RATE));
  tc_set_overflow_interrupt_callback(&PROTOCOL_TIMER, protocol_timer_overflow_handler);
  tc_set_overflow_interrupt_level(&PROTOCOL_TIMER, TC_INT_LVL_MED); 
}

static void transceiver_task_loop(void *p) {
  init_nrf24l01p(); 

  nrf24l01p_set_interrupt_mask(NRF24L01P_INTR_TX_DS);
  nrf24l01p_set_interrupt_pin_handler(nrf24l01p_interrupt_handler);

  for(;;) {
    state = TRANSCEIVER_STATE_IDLE;
    vTaskSuspend(NULL); // Suspend until we have to do something...

    switch(state) {
      case TRANSCEIVER_STATE_TX_PREP:
        tx_prep_operation();
        break;
      case TRANSCEIVER_STATE_TRANSMITTING:
        transmit_operation();
        break;
      case TRANSCEIVER_STATE_RECEIVING:
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

static void tx_prep_operation(void) {
  // Swap out the queue that other tasks are writing to
  transceiver_send_queue = using_queue_a ? &send_queue_b : &send_queue_a;
  QueueHandle_t *queue_to_prep = !using_queue_a ? &send_queue_b : &send_queue_a;
  using_queue_a = !using_queue_a;
  
  uint8_t frame_len = uxQueueMessagesWaiting(*queue_to_prep) + 1;
  protocol_raw_frame_t *frame = pvPortMalloc(sizeof(protocol_raw_frame_t));
  frame->len = frame_len;
  frame->packets = pvPortMalloc(frame_len * sizeof(protocol_packet_t *));

  // Prepare Header
  uint8_t index = 0;
  frame->packets[index] = (protocol_packet_t *)pvPortMalloc(sizeof(protocol_packet_t));
  frame->packets[index]->index = index;
  frame->packets[index]->len = 1;
  frame->packets[index]->bytes = (uint8_t *)pvPortMalloc(sizeof(uint8_t));
  frame->packets[index]->bytes[0] = frame_len;
  index++;

  // Prepare Body
  while(uxQueueMessagesWaiting(*queue_to_prep)) {
    struct transceiver_message message;
    xQueueReceive(*queue_to_prep, (void *)&message, 0);

    switch(message.type) {
      case TRANSCEIVER_MSG_TYPE_GENERAL:
        frame->packets[index] = general_message_to_packet((general_message_t *)message.data);
        frame->packets[index]->index = index;
        break;
    }
  }
}

static void transmit_operation(void) {
  nrf24l01p_set_radio_mode(NRF24L01P_MODE_TX);

}

static protocol_packet_t *general_message_to_packet(general_message_t *message) {
  protocol_packet_t *packet = (protocol_packet_t *)pvPortMalloc(sizeof(protocol_packet_t));
  packet->bytes = (uint8_t *)message->text;
  packet->len = message->len;
  packet->index = 0;
  return packet;
}

static void dma_xfer_complete_handler(void) {
  xTaskResumeFromISR(transceiver_task_handle);
}

static void nrf24l01p_interrupt_handler(void) {

}

static void protocol_timer_overflow_handler(void) {
  // We need to initiate a transmit frame
  state = TRANSCEIVER_STATE_TRANSMITTING;
  xTaskResumeFromISR(transceiver_task_handle);
}
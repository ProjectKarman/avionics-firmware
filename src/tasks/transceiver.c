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

#define EVENT_QUEUE_DEPTH 10

enum transeiver_state {
  TRANSCEIVER_STATE_IDLE,
  TRANSCEIVER_STATE_SLEEP,
  TRANSCEIVER_STATE_TRANSMITTING,
  TRANSCEIVER_STATE_RECEIVING,
};

enum transceiver_event_type {
  TRANSCEIVER_EVENT_MESSAGE_SENT,
  TRANSCEIVER_EVENT_TX_FRAME_COMPLETE
};

typedef struct {
  enum transceiver_event_type type;
  void *data;
} transceiver_event_t;

static void transceiver_task_loop(void *p);
static void init_nrf24l01p(void);

static void add_message_to_frame(transceiver_message_t *new_message);

static protocol_packet_t *general_message_to_packet(general_message_t *message);

static void dma_xfer_complete_handler(void);
static void nrf24l01p_interrupt_handler(void);
static void protocol_timer_overflow_handler(void);
static void protocol_timer_cc_match_handler(void);

TaskHandle_t transceiver_task_handle;

static QueueHandle_t event_queue;
static volatile enum transeiver_state state;
static packet_history_t *packet_history;
static uint8_t currentPacketIndex;
static uint8_t fifo_fill_depth;

void transceiver_start_task(void) {
  xTaskCreate(transceiver_task_loop, "transceiver", 200, NULL, 2, &transceiver_task_handle);
  event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(transceiver_event_t));

  packet_history = pvPortMalloc(sizeof(packet_history_t));
  packet_history_init(packet_history);

  /*
  // Configure timer to generate frame phase interrupts
  tc_enable(&PROTOCOL_TIMER);
  tc_set_wgm(&PROTOCOL_TIMER, TC_WG_NORMAL);
  tc_write_period(&PROTOCOL_TIMER, F_CPU / (4 * PROTOCOL_FRAME_RATE));
  tc_write_cc(&PROTOCOL_TIMER, TC_CCA, 24000); // TODO: Come up with a general eqn
  tc_enable_cc_channels(&PROTOCOL_TIMER, TC_CCAEN);
  tc_set_overflow_interrupt_callback(&PROTOCOL_TIMER, protocol_timer_overflow_handler);
  tc_set_overflow_interrupt_level(&PROTOCOL_TIMER, TC_INT_LVL_MED);
  tc_set_cca_interrupt_callback(&PROTOCOL_TIMER, protocol_timer_cc_match_handler);
  tc_set_cca_interrupt_level(&PROTOCOL_TIMER, TC_INT_LVL_MED);
  tc_write_clock_source(&PROTOCOL_TIMER, TC_CLKSEL_DIV4_gc);
  */
}

void transceiver_send_message(transceiver_message_t *message, TickType_t ticks_to_wait) {
  transceiver_event_t event = {
    .data = message,
    .type = TRANSCEIVER_EVENT_MESSAGE_SENT
  };
  xQueueSendToBack(event_queue, &event, ticks_to_wait);
}

static void transceiver_task_loop(void *p) {
  init_nrf24l01p(); 

  nrf24l01p_set_interrupt_mask(NRF24L01P_INTR_TX_DS);
  nrf24l01p_set_interrupt_pin_handler(nrf24l01p_interrupt_handler);

  // Make up data
  uint8_t test_data[32], i;
  for(i = 0; i < 32; i++) {
    test_data[i] = i;
  }
  uint8_t fifo_status;
  nrf24l01p_read_register(0x17, &fifo_status);
  nrf24l01p_init_tx_payload_xfer(test_data, 32, dma_xfer_complete_handler);
  vTaskSuspend(NULL);
  nrf24l01p_read_register(0x17, &fifo_status);
  nrf24l01p_init_tx_payload_xfer(test_data, 32, dma_xfer_complete_handler);
  vTaskSuspend(NULL);
  nrf24l01p_read_register(0x17, &fifo_status);
  nrf24l01p_init_tx_payload_xfer(test_data, 32, dma_xfer_complete_handler);
  vTaskSuspend(NULL);
  nrf24l01p_read_register(0x17, &fifo_status);

  transceiver_event_t currentEvent;
  for(;;) {
    xQueueReceive(event_queue, &currentEvent, portMAX_DELAY);
    
    switch(currentEvent.type) {
      case TRANSCEIVER_EVENT_MESSAGE_SENT:
        add_message_to_frame((transceiver_message_t *)currentEvent.data);
        break;
      case TRANSCEIVER_EVENT_TX_FRAME_COMPLETE:
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

static void add_message_to_frame(transceiver_message_t *new_message) {
  switch(new_message->type) {
    case TRANSCEIVER_MSG_TYPE_GENERAL:
      //frame->packets[index] = general_message_to_packet((general_message_t *)new_message->data);
      //frame->packets[index]->index = index;
      break;
  }
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
  /*
  switch(state) {
    case TRANSCEIVER_STATE_TRANSMITTING:
      // DMA Payload transfer just completed

      if(currentPacketIndex == 0) {
        // Enable transmitter
      }

      fifo_fill_depth++;
      if(fifo_fill_depth < 3) {
        // TODO: Initiate another DMA transfer
      }
      break;
    case TRANSCEIVER_STATE_RECEIVING:
      break;
    default:
      break;
  }
  */
}

static void nrf24l01p_interrupt_handler(void) {
  switch(state) {
    case TRANSCEIVER_STATE_TRANSMITTING:
      // Packet transmitted
      fifo_fill_depth--;
      break;
    case TRANSCEIVER_STATE_RECEIVING:
      // Packet received
      fifo_fill_depth++;
      break;
    default:
      break;
  }
}

static void protocol_timer_overflow_handler(void) {
  // Start downlink frame transmission timer interrupt
  // Send transmit event
}

static void protocol_timer_cc_match_handler(void) {

}
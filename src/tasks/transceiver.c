/*
 * transceiver.c
 *
 * Created: 10/8/2015 5:20:31 PM
 *  Author: Nigil Lee
 */ 

#include <stdbool.h>
#include <string.h>
#include "asf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "nrf24l01p.h"
#include "downlink_frame.h"
#include "message_types.h"
#include "transceiver.h"

#define PROTOCOL_TIMER TCC1
#define PROTOCOL_FRAME_RATE 250

#define PACKET_BYTES 20

#define EVENT_QUEUE_DEPTH 5

enum transeiver_state {
  TRANSCEIVER_STATE_IDLE,
  TRANSCEIVER_STATE_SLEEP,
  TRANSCEIVER_STATE_TRANSMITTING,
  TRANSCEIVER_STATE_RECEIVING,
  TRANSCEIVER_STATE_PREPARING
};

enum transceiver_event_type {
  TRANSCEIVER_EVENT_MESSAGE_SENT,
  TRANSCEIVER_EVENT_TX_FRAME_COMPLETE,
  TRANSCEIVER_EVENT_TX_FRAME_PREPARE
};

typedef struct {
  enum transceiver_event_type type;
  void *data;
} transceiver_event_t;

static void transceiver_task_loop(void *p);
static void init_nrf24l01p(void);
static void init_timer(void);
static void add_priority_event(enum transceiver_event_type event_type, void *data);
static void prepare_transmit_frame(void);
static void add_message_to_frame(transceiver_message_t *new_message);
static downlink_packet_t *general_message_to_packet(general_message_t *message);
static void prepare_transmit_frame(void);

static void dma_xfer_complete_handler(void);
static void nrf24l01p_interrupt_handler(void);
static void nrf24l01p_reset_interrupt_handler(void);
static void protocol_timer_overflow_handler(void);
static void protocol_timer_cc_match_handler(void);

TaskHandle_t transceiver_task_handle;

static QueueHandle_t event_queue;
static volatile enum transeiver_state state;
static volatile uint8_t fifo_fill_depth;
static downlink_frame_t *currently_building_frame;
static downlink_frame_t *frame_to_send;

void transceiver_start_task(void) {
  xTaskCreate(transceiver_task_loop, "transceiver", 200, NULL, 2, &transceiver_task_handle);
  event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(transceiver_event_t));
}

void transceiver_send_message(transceiver_message_t *message, TickType_t ticks_to_wait) {
  transceiver_event_t event = {
    .data = message,
    .type = TRANSCEIVER_EVENT_MESSAGE_SENT
  };
  xQueueSendToBack(event_queue, &event, ticks_to_wait);
}

transceiver_message_t *transceiver_message_create(void)
{
  transceiver_message_t *message = (transceiver_message_t *)pvPortMalloc(sizeof(transceiver_message_t));
  if(message != NULL) {
    memset(message, 0, sizeof(transceiver_message_t));
  }
  return message;
}

void transceiver_message_destroy(transceiver_message_t *message) {
  vPortFree(message);
}

static void transceiver_task_loop(void *p) {
  init_nrf24l01p(); 
  init_timer();

  currently_building_frame = downlink_frame_create();

  transceiver_event_t currentEvent;
  for(;;) {
    xQueueReceive(event_queue, &currentEvent, portMAX_DELAY);
    
    switch(currentEvent.type) {
      case TRANSCEIVER_EVENT_MESSAGE_SENT:
        add_message_to_frame((transceiver_message_t *)currentEvent.data);
        break;
      case TRANSCEIVER_EVENT_TX_FRAME_COMPLETE:
        downlink_frame_destory(frame_to_send);
        state = TRANSCEIVER_STATE_IDLE;

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
        break;
      case TRANSCEIVER_EVENT_TX_FRAME_PREPARE:
        prepare_transmit_frame();
        break;
    }
  }
}

static void init_nrf24l01p(void) {
  char *address = "W1KBN";

  //nrf24l01p_read_regs();
  nrf24l01p_wake();
  nrf24l01p_flush_rx_fifo();
  nrf24l01p_flush_tx_fifo();
  nrf24l01p_reset_interrupts();
  nrf24l01p_set_channel(20);
  nrf24l01p_set_data_rate(NRF24L01P_DR_2M);
  nrf24l01p_set_pa_power(NRF24L01P_PWR_N18DBM);
  nrf24l01p_set_interrupt_mask(NRF24L01P_INTR_MAX_RT);
  nrf24l01p_set_autoack_mask(0x0);
  nrf24l01p_set_retransmission(0, 0);
  nrf24l01p_set_address((uint8_t *)address, 5);
  nrf24l01p_set_interrupt_pin_handler(nrf24l01p_interrupt_handler);
}

static void init_timer(void) {
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
}

static inline void add_priority_event(enum transceiver_event_type event_type, void *data) {
  transceiver_event_t event = {
    .data = data,
    .type = event_type
  };
  xQueueSendToFrontFromISR(event_queue, &event, NULL);
};

static void prepare_transmit_frame(void) {
  state = TRANSCEIVER_STATE_PREPARING;

  frame_to_send = currently_building_frame;
  currently_building_frame = downlink_frame_create();
  downlink_frame_prepare_for_sending(frame_to_send);
  downlink_packet_t *first_packet = downlink_frame_get_packet(frame_to_send);
  nrf24l01p_send_payload_async(first_packet->bytes, first_packet->len, dma_xfer_complete_handler);
}

static void add_message_to_frame(transceiver_message_t *new_message) {
  switch(new_message->type) {
    case TRANSCEIVER_MSG_TYPE_GENERAL:
      downlink_frame_add_packet(currently_building_frame ,general_message_to_packet((general_message_t *)new_message->data));
      transceiver_message_destroy(new_message);
      break;
  }
  
}

/*
 * The following section of functions converts specific message types to packets that can be
 *  sent by the transceiver.
 *
 * Based on the protocol format we need to use pointer arithmetic to offset the copy so 
 *  downlink frame function can insert their packet index
 */

static downlink_packet_t *general_message_to_packet(general_message_t *message) {
  downlink_packet_t *packet = downlink_packet_create();

  packet->bytes = pvPortMalloc(sizeof(uint8_t) * (message->len + 1));
  packet->len = message->len + 1;

  memcpy(packet->bytes + 1, message->text, message->len);

  general_message_destory(message);
  
  return packet;
}

static void dma_xfer_complete_handler(void) {
  switch(state) {
    case TRANSCEIVER_STATE_PREPARING:
      fifo_fill_depth++;
      if(fifo_fill_depth < 3) {
        downlink_packet_t *packet = downlink_frame_get_packet(frame_to_send);
        if(packet != NULL) {
          nrf24l01p_send_payload_async(packet->bytes, packet->len, dma_xfer_complete_handler);
        }
      }
      break;
    case TRANSCEIVER_STATE_TRANSMITTING:
      // DMA Payload transfer just completed
      fifo_fill_depth++;      
      break;
    case TRANSCEIVER_STATE_RECEIVING:
      break;
    default:
      break;
  }
}

static void nrf24l01p_interrupt_handler(void) {
  nrf24l01p_reset_interrupts_async(nrf24l01p_reset_interrupt_handler);
}

static void nrf24l01p_reset_interrupt_handler(void) {
  switch(state) {
    case TRANSCEIVER_STATE_TRANSMITTING:
      // Packet transmitted
      fifo_fill_depth--;
      downlink_packet_t *packet = downlink_frame_get_packet(frame_to_send);
      if(packet != NULL) {
        nrf24l01p_send_payload_async(packet->bytes, packet->len, dma_xfer_complete_handler);
      }
      else if(fifo_fill_depth == 0) {
        // No more packets to transmit
        state = TRANSCEIVER_STATE_IDLE;
        nrf24l01p_end_operation();
        add_priority_event(TRANSCEIVER_EVENT_TX_FRAME_COMPLETE, NULL);
      }
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
  nrf24l01p_start_operation();
  state = TRANSCEIVER_STATE_TRANSMITTING;
}

static void protocol_timer_cc_match_handler(void) {
  // We need to set everything up to be able to transmit
  add_priority_event(TRANSCEIVER_EVENT_TX_FRAME_PREPARE, NULL);
}
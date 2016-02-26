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

#define EVENT_QUEUE_DEPTH 8
#define EVENT_DATA_SIZE_MAX 25

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

// The following sturct uses a buffer that can be cast to the correct type base on "type" variable
typedef struct {
  enum transceiver_event_type type;
  uint8_t data[EVENT_DATA_SIZE_MAX];
} transceiver_event_t;

static void transceiver_task_loop(void *p);
static void init_nrf24l01p(void);
static void init_timer(void);
static void add_priority_event(enum transceiver_event_type event_type, void *data);
static void prepare_transmit_frame(void);
static void add_message_to_frame(transceiver_message_t *new_message);
static downlink_packet_t general_message_to_packet(general_message_t *message);
static void prepare_transmit_frame(void);

static void nrf24l01p_interrupt_handler(void);
static void protocol_timer_overflow_handler(void);
static void protocol_timer_cc_match_handler(void);

TaskHandle_t transceiver_task_handle;

static QueueHandle_t event_queue;
static volatile enum transeiver_state state;
static volatile uint8_t fifo_fill_depth;
static volatile bool is_interrupt_reset_defered;
static downlink_frame_t *currently_building_frame;
static downlink_frame_t *frame_to_send;
static downlink_frame_t frame_1;
static downlink_frame_t frame_2;

static uint16_t reset_time;

void transceiver_start_task(void) {
  xTaskCreate(transceiver_task_loop, "transceiver", 1536, NULL, 2, &transceiver_task_handle);
  event_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(transceiver_event_t));
}

void transceiver_send_message(transceiver_message_t message, TickType_t ticks_to_wait) {
  transceiver_event_t event = {
    .type = TRANSCEIVER_EVENT_MESSAGE_SENT
  };
  memcpy(event.data, &message, EVENT_DATA_SIZE_MAX);
  xQueueSendToBack(event_queue, &event, ticks_to_wait);
}

transceiver_message_t transceiver_message_create(enum transceiver_message_type type, void *contents) {
  transceiver_message_t message = {
    .type = type,
  };
  memcpy(message.data, contents, MESSAGE_MAX_SIZE);
  return message;
}

static void transceiver_task_loop(void *p) {
  vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
  
  // Config RF Frontend
  PORTK.DIR |= 0x7c;
  PORTK.OUT |= 0x30;
  
  /*
  nrf24l01p_wake();
  nrf24l01p_generate_carrier();
  nrf24l01p_set_pa_power(NRF24L01P_PWR_0DBM);
  nrf24l01p_set_channel(20);
  nrf24l01p_start_operation();
  
  for(;;) {
    vTaskDelay(10);
  }
  */ 
  
  init_nrf24l01p();
  init_timer();

  currently_building_frame = &frame_1;
  frame_to_send = &frame_2;
  downlink_frame_init(&frame_1);

  transceiver_event_t currentEvent;
  for(;;) {
    xQueueReceive(event_queue, &currentEvent, portMAX_DELAY);
    
    switch(currentEvent.type) {
      case TRANSCEIVER_EVENT_MESSAGE_SENT:
        add_message_to_frame((transceiver_message_t *)currentEvent.data);
        break;
      case TRANSCEIVER_EVENT_TX_FRAME_COMPLETE:
        state = TRANSCEIVER_STATE_IDLE;

        if(reset_time == 1000) {
          CPU_CCP = 0xD8;
          RST.CTRL |= 0x1;
        }
        else {
          reset_time++;
        }

        general_message_t content = {
          .text = "Hello World!",
          .len = 12
        };

        transceiver_message_t message = transceiver_message_create(TRANSCEIVER_MSG_TYPE_GENERAL, &content);
        
        transceiver_send_message(message, 0);
        transceiver_send_message(message, 0);
        transceiver_send_message(message, 0);
        transceiver_send_message(message, 0);
        transceiver_send_message(message, 0);
        break;
      case TRANSCEIVER_EVENT_TX_FRAME_PREPARE:
        prepare_transmit_frame();
        break;
    }
  }
}

static void init_nrf24l01p(void) {
  char *address = "W1KBN";

  nrf24l01p_wake();
  nrf24l01p_flush_rx_fifo();
  nrf24l01p_flush_tx_fifo();
  nrf24l01p_reset_interrupts();
  nrf24l01p_set_channel(50);
  nrf24l01p_set_data_rate(NRF24L01P_DR_2M);
  nrf24l01p_set_pa_power(NRF24L01P_PWR_0DBM);
  nrf24l01p_set_interrupt_mask(NRF24L01P_INTR_MAX_RT);
  nrf24l01p_set_autoack_mask(0x0);
  nrf24l01p_set_retransmission_count(0);
  nrf24l01p_set_retransmission_delay(0);
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
    .type = event_type
  };
  memcpy(event.data, data, EVENT_DATA_SIZE_MAX);
  xQueueSendToFrontFromISR(event_queue, &event, NULL);
};

static void prepare_transmit_frame(void) {
  state = TRANSCEIVER_STATE_PREPARING;
  // TODO Make atomic
  downlink_frame_t *tmp = currently_building_frame;
  currently_building_frame = frame_to_send;
  frame_to_send = tmp;
  downlink_frame_init(currently_building_frame);

  downlink_frame_write_header(frame_to_send);
  downlink_packet_t packet;

  for(uint8_t i; i < 3; i++) {
    if(downlink_frame_get_next_packet(frame_to_send, &packet)) {
      break;
    }
    nrf24l01p_send_payload(packet.bytes, packet.len);
    fifo_fill_depth++;
  }
}

static void add_message_to_frame(transceiver_message_t *new_message) {
  downlink_packet_t packet;
  switch(new_message->type) {
    case TRANSCEIVER_MSG_TYPE_GENERAL:
      packet = general_message_to_packet((general_message_t *)new_message->data);
      
      break;
  }
  downlink_frame_add_packet(currently_building_frame, &packet);
}

/*
 * The following section of functions converts specific message types to packets that can be
 *  sent by the transceiver.
 *
 * Based on the protocol format we need to use pointer arithmetic to offset the copy so 
 *  downlink frame function can insert their packet index
 */

static downlink_packet_t general_message_to_packet(general_message_t *message) {
  downlink_packet_t packet;
  memcpy(packet.bytes + 2, message->text, message->len); // 0 = DMA command space, 1 = packet number
  packet.len = message->len + 1;
  return packet;
}

static void nrf24l01p_interrupt_handler(void) {
  switch(state) {
    case TRANSCEIVER_STATE_TRANSMITTING:
    // Packet transmitted
    fifo_fill_depth--;

    downlink_packet_t packet;
    if(!downlink_frame_get_next_packet(frame_to_send, &packet)) {
      nrf24l01p_reset_interrupts_and_send_payload_from_isr(packet.bytes, packet.len);
      fifo_fill_depth++;
    }
    else if(fifo_fill_depth == 0) {
      // No more packets to transmit
      nrf24l01p_reset_interrupts();
      state = TRANSCEIVER_STATE_IDLE;
      nrf24l01p_end_operation();
      add_priority_event(TRANSCEIVER_EVENT_TX_FRAME_COMPLETE, NULL);
    }
    else {
      nrf24l01p_reset_interrupts();
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
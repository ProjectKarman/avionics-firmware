/*
 * sensor.c
 *
 * Created: 11/19/2015 8:59:23 PM
 *  Author: Nigil Lee
 */

#include "sensor.h"

#include "asf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ms5607_02ba.h"
// #include "fxls8471qr1.h"
// #include "l3g4200d.h"
#include "nrf24l01p.h"
#include "message_types.h"
#include "transceiver.h"

/* Constant Variables */
#define SESNORCOLLECTION_100Hz TCC2
#define SESNORCOLLECTION_400Hz TCD2
#define SESNORCOLLECTION_800Hz TCE2
#define EVENT_DATA_SIZE_MAX 25
#define EVENT_QUEUE_DEPTH 8
#define BASE_HERTZ_MODIFIER 40000

enum sensor_queue_state_type {
  SENSOR_ENTRY_100Hz,
  SENSOR_ENTRY_400Hz,
  SENSOR_ENTRY_800Hz
};

typedef struct {
  enum sensor_queue_state_type type;
  uint8_t data[EVENT_DATA_SIZE_MAX];
} sensor_timer_t;

static QueueHandle_t sensor_queue;

/* Private Prototypes */
static void sensor_task_loop();
static void sensor_initialize();
static void startup_timer();
static void send_to_tranceiver();
static void protocol_timer1_overflow_handler();
static void protocol_timer2_overflow_handler();
static void protocol_timer3_overflow_handler();
static inline void add_priority_event(enum sensor_queue_state_type event_type, void *data);

TaskHandle_t sensor_task_handle;
static QueueHandle_t sensor_queue;
sensors_message_t current_sensor_readings;

void sensor_start_task(void) {
  xTaskCreate(sensor_task_loop, "sensor", 1024, NULL, 2, &sensor_task_handle);
  sensor_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(sensor_timer_t));
}

/* Private Functions */
static void sensor_task_loop() {
  TaskHandle_t sensor_task_handle;
  sensor_timer_t timer_update;

  memset(&current_sensor_readings, 0, sizeof(sensors_message_t));

  sensor_initialize();
  startup_timer();

  for(;;) {
    xQueueReceive(sensor_queue, &timer_update, portMAX_DELAY);

    switch(timer_update.type) {
      case SENSOR_ENTRY_800Hz:
        break;
      case SENSOR_ENTRY_400Hz:
        break;
      case SENSOR_ENTRY_100Hz:
        break;
    }
  send_to_tranceiver();
  }
}

static void sensor_initialize() {
  ms5607_02ba_reset();
  ms5607_02ba_load_prom();

  // fxls8471qr1_init(void);
  // l3g4200d_init();

  nrf24l01p_init();
}

static void startup_timer()  {
  // Configure timer to generate frame phase interrupts
  tc_enable(&SESNORCOLLECTION_100Hz);
  tc_enable(&SESNORCOLLECTION_400Hz);
  tc_enable(&SESNORCOLLECTION_800Hz);

  tc_set_wgm(&SESNORCOLLECTION_100Hz, TC_WG_NORMAL);
  tc_set_wgm(&SESNORCOLLECTION_400Hz, TC_WG_NORMAL);
  tc_set_wgm(&SESNORCOLLECTION_800Hz, TC_WG_NORMAL);

  tc_write_period(&SESNORCOLLECTION_100Hz, F_CPU / (8 * BASE_HERTZ_MODIFIER));
  tc_write_period(&SESNORCOLLECTION_400Hz, F_CPU / (2 * BASE_HERTZ_MODIFIER));
  tc_write_period(&SESNORCOLLECTION_800Hz, F_CPU / (BASE_HERTZ_MODIFIER));

  tc_set_overflow_interrupt_callback(&SESNORCOLLECTION_100Hz, protocol_timer1_overflow_handler);
  tc_set_overflow_interrupt_callback(&SESNORCOLLECTION_400Hz, protocol_timer2_overflow_handler);
  tc_set_overflow_interrupt_callback(&SESNORCOLLECTION_800Hz, protocol_timer3_overflow_handler);

  tc_set_overflow_interrupt_level(&SESNORCOLLECTION_100Hz, TC_INT_LVL_MED);
  tc_set_overflow_interrupt_level(&SESNORCOLLECTION_400Hz, TC_INT_LVL_MED);
  tc_set_overflow_interrupt_level(&SESNORCOLLECTION_800Hz, TC_INT_LVL_MED);
}

static void send_to_tranceiver() {
  transceiver_message_t created_message;

  // format message from struct based on message
  created_message = transceiver_message_create(TRANSCEIVER_MSG_TYPE_GENERAL, &current_sensor_readings);

  // transceiver_send_message/transceiver_message_create
  transceiver_send_message(created_message, 0);
}

static void protocol_timer1_overflow_handler() {
  add_priority_event(SENSOR_ENTRY_100Hz, NULL);
}

static void protocol_timer2_overflow_handler() {
  add_priority_event(SENSOR_ENTRY_400Hz, NULL);
}

static void protocol_timer3_overflow_handler() {
  add_priority_event(SENSOR_ENTRY_800Hz, NULL);
}

static inline void add_priority_event(enum sensor_queue_state_type event_type, void *data) {
  sensor_timer_t event = {
    .type = event_type
  };
  memcpy(event.data, data, EVENT_DATA_SIZE_MAX);
  xQueueSendToFrontFromISR(sensor_queue, &event, NULL);
};

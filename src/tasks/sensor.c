/*
 * sensor.c
 *
 * Created: 11/19/2015 8:59:23 PM
 *  Authors: Bryce Carter, Nigil Lee, Tim Rupprecht
 */ 

#include "sensor.h"

#include "asf.h"
//#include "tc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "ms5607_02ba_dev.h"
// #include "fxls8471qr1.h"
// #include "l3g4200d.h"
#include "nrf24l01p.h"
#include "message_types.h"
#include "transceiver.h"
#include "twi_interface.h"

/* Constant Variables */
#define SENSOR_TIMER TCE2

#define EVENT_DATA_SIZE_MAX 25
#define EVENT_QUEUE_DEPTH 8
#define BASE_HERTZ_MODIFIER 40000

enum sensor_queue_state_type {
	SENSOR_ENTRY_100Hz,
	SENSOR_ENTRY_400Hz,
	SENSOR_ENTRY_800Hz,
	SENSOR_ENTRY_NONE,
	SENSOR_CHECK_TWI_3200Hz
};

typedef struct {
	enum sensor_queue_state_type type;
	uint8_t data[EVENT_DATA_SIZE_MAX];
} sensor_timer_t;

extern twi_interface_t twie;
extern ms5607_02ba_dev_t ms5607_02ba;

static QueueHandle_t sensor_queue;

/* Private Prototypes */
static void sensor_task_loop();
static void sensor_initialize();
static void startup_timer();
static void send_to_tranceiver();

static void protocol_timer_overflow_handler();

static inline void add_priority_event(enum sensor_queue_state_type event_type, void *data);

TaskHandle_t sensor_task_handle;
static QueueHandle_t sensor_queue;
sensors_message_t current_sensor_readings;

int8_t get_ms5607_data_countdown = -1; // number of 3.2kHz ticks remaining

void sensor_start_task(void) {
  xTaskCreate(sensor_task_loop, "sensor", 1024, NULL, 2, &sensor_task_handle);
  sensor_queue = xQueueCreate(EVENT_QUEUE_DEPTH, sizeof(sensor_timer_t));
}

/* Private Functions */
static void sensor_task_loop() {
  TaskHandle_t sensor_task_handle;
  sensor_timer_t timer_update;
  
  memset(&current_sensor_readings, 0, sizeof(sensors_message_t));
  
  // TODO: Originally Bryce was trying to 
  // twi_interface_t *twie_interface;
  // twie_interface = &twie;
  
  sensor_initialize();
  startup_timer();
  ms5607_02ba_reset();
  ms5607_02ba_load_prom();

  for(;;) {
    xQueueReceive(sensor_queue, &timer_update, NULL);

    switch(timer_update.type) {
	  case SENSOR_CHECK_TWI_3200Hz:
		  if (get_ms5607_data_countdown >= 0)
		  {
			  get_ms5607_data_countdown -= 1;
			  if(get_ms5607_data_countdown == 0)
			  {
				  xQueueSendToBack(twie.twi_todo_queue, &ms5607_02ba.prepareADC, NULL);
				  xQueueSendToBack(twie.twi_todo_queue, &ms5607_02ba.getADC, NULL);
			  }
		  }
			// Add more countdowns here
		break;
      case SENSOR_ENTRY_800Hz:
        break;
	  case SENSOR_ENTRY_400Hz:
	  	xQueueSendToBack(twie.twi_todo_queue, &ms5607_02ba.prepareD2, NULL);
		get_ms5607_data_countdown = 26; // this might not work
	    break;	
      case SENSOR_ENTRY_100Hz:
        break;
    }
    timer_update.type = SENSOR_ENTRY_NONE;
    if (ms5607_02ba_fetch_queue_data())
	{
		// send_to_tranceiver();
	}
	twi_process_queue();
  }
}  

static void sensor_initialize() {
  // ms5607_02ba_init();
  
  // fxls8471qr1_init(void);
  // l3g4200d_init();
  
  // nrf24l01p_init();
}

static void startup_timer()
{
	tc_enable(&SENSOR_TIMER);
	tc_set_wgm(&SENSOR_TIMER, TC_WG_NORMAL);
	tc_write_period(&SENSOR_TIMER, F_CPU / (32 * BASE_HERTZ_MODIFIER));
	
	tc_set_overflow_interrupt_callback(&SENSOR_TIMER, protocol_timer_overflow_handler);
	tc_set_overflow_interrupt_level(&SENSOR_TIMER, TC_INT_LVL_LO);
	
	tc_write_clock_source(&SENSOR_TIMER, TC_CLKSEL_DIV1_gc);
}

static void send_to_tranceiver() {
  transceiver_message_t created_message;
	
  // format message from struct based on message
  created_message = transceiver_message_create(TRANSCEIVER_MSG_TYPE_GENERAL, &current_sensor_readings);
	 
  // transceiver_send_message/transceiver_message_create
  transceiver_send_message(created_message, 0);
}

static void protocol_timer_overflow_handler() {
	static uint8_t hertz_state = 0;
	
	if (hertz_state == 0) {
		add_priority_event(SENSOR_ENTRY_100Hz, NULL);
		add_priority_event(SENSOR_ENTRY_400Hz, NULL);
		add_priority_event(SENSOR_ENTRY_800Hz, NULL);
	}
	else if ((hertz_state & 7) == 0) {
		add_priority_event(SENSOR_ENTRY_400Hz, NULL);
	}
	else if ((hertz_state & 3) == 0)
	{
		add_priority_event(SENSOR_ENTRY_800Hz, NULL);
	}
	add_priority_event(SENSOR_CHECK_TWI_3200Hz, NULL);

	
	if (hertz_state == 31)
	{
		hertz_state = 0;
	}
	else{
		hertz_state+=1;
	}
}

static inline void add_priority_event(enum sensor_queue_state_type event_type, void *data) {
	sensor_timer_t event = {
		.type = event_type
	};
	memcpy(event.data, data, EVENT_DATA_SIZE_MAX);
	xQueueSendToFront(sensor_queue, &event, NULL);
};
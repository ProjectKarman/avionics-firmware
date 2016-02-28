/*
 * sensor.c
 *
 * Created: 11/19/2015 8:59:23 PM
 *  Author: Nigil Lee
 */ 

#include "sensor.h"

#include "asf.h"
//#include "tc.h"
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
#define SENSOR_TIMER TCE2

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

static void protocol_timer_overflow_handler();

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
  static uint8_t temp_debug_variable_breakpoint = 0;
  
  memset(&current_sensor_readings, 0, sizeof(sensors_message_t));
  
  sensor_initialize();
  startup_timer();
  ms5607_02ba_reset();
  ms5607_02ba_load_prom();
  
  temp_debug_variable_breakpoint = 10;
  
  
  for(;;) {
    xQueueReceive(sensor_queue, &timer_update, portMAX_DELAY);
	temp_debug_variable_breakpoint += 1; 
    switch(timer_update.type) {
      case SENSOR_ENTRY_800Hz:
		temp_debug_variable_breakpoint += 1;
        break;
	  case SENSOR_ENTRY_400Hz:
		temp_debug_variable_breakpoint += 2;
	    break;	
      case SENSOR_ENTRY_100Hz:
		temp_debug_variable_breakpoint += 3;
        break;
    }
	// send_to_tranceiver();
	temp_debug_variable_breakpoint += 1;
  }
}  

static void sensor_initialize() {
  ms5607_02ba_init();
  
  // fxls8471qr1_init(void);
  // l3g4200d_init();
  
  // nrf24l01p_init();
}

/*
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
*/

static void startup_timer()
{
	/*
	 SENSOR_TIMER.CNT = 0;// Zeroise count
	 SENSOR_TIMER.PER = 800; //Period
	 SENSOR_TIMER.CTRLA = TC_CLKSEL_DIV1_gc; //Divider
	 SENSOR_TIMER.INTCTRLA = TC_OVFINTLVL_LO_gc; //Liow level interrupt
	 SENSOR_TIMER.INTFLAGS = 0x01; // clear any initial interrupt flags
	 SENSOR_TIMER.CTRLB = TC_WGMODE_NORMAL_gc; // Normal operation
	 
	 PMIC.CTRL |= PMIC_LOLVEN_bm;
	 */
	
	tc_enable(&SENSOR_TIMER);
	tc_set_wgm(&SENSOR_TIMER, TC_WG_NORMAL);
	tc_write_period(&SENSOR_TIMER, F_CPU / (8 * BASE_HERTZ_MODIFIER));
	
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

static void protocol_timer1_overflow_handler() {
	add_priority_event(SENSOR_ENTRY_100Hz, NULL);
}

static void protocol_timer2_overflow_handler() {
	add_priority_event(SENSOR_ENTRY_400Hz, NULL);
}

static void protocol_timer3_overflow_handler() {
	add_priority_event(SENSOR_ENTRY_800Hz, NULL);
}

static void protocol_timer_overflow_handler() {
	static uint8_t hertz_state = 0;
	
	if (hertz_state == 0) {
		add_priority_event(SENSOR_ENTRY_100Hz, NULL);
		add_priority_event(SENSOR_ENTRY_400Hz, NULL);
	}
	else if ((hertz_state & 1) == 0) {
		add_priority_event(SENSOR_ENTRY_400Hz, NULL);
	}
	add_priority_event(SENSOR_ENTRY_800Hz, NULL);
	
	if (hertz_state == 7)
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
	// xQueueSendToFrontFromISR(sensor_queue, &event, NULL);
	xQueueSendToFront(sensor_queue, &event, NULL);
};
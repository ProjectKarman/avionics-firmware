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
// #include "hmc5883l.h"
// #include "fxls8471qr1.h"
// #include "l3g4200d.h"
#include "Si7021-A20.h"
#include "nrf24l01p.h"
#include "hmc5883l.h"
#include "message_types.h"
#include "transceiver.h"
#include "twi_interface.h"

/* Constant Variables */
#define SENSOR_TIMER TCE2

#define SESNORCOLLECTION_100Hz TCC2
#define SESNORCOLLECTION_400Hz TCD2
#define SESNORCOLLECTION_800Hz TCE2

#define EVENT_DATA_SIZE_MAX 25
#define EVENT_QUEUE_DEPTH 8
#define BASE_HERTZ_MODIFIER 40000

enum sensor_queue_state_type {
	SENSOR_ENTRY_50Hz,
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
static void sensor_task_loop(void * pvParameters);
static void sensor_initialize(void);
static void startup_timer(void);
// static void send_to_tranceiver(void);

static void protocol_timer_overflow_handler(void);

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
static void sensor_task_loop(void * pvParameters)
{
  sensor_timer_t timer_update;
  MS5607_02BA_POLL_MODE_T altimeter_poll_mode = START_UP;

  memset(&current_sensor_readings, 0, sizeof(sensors_message_t));

  // Setup hardware interfaces
  // ============================
  twi_init();
  // ============================

  //Setup sensor drivers
  // ============================
  sensor_initialize();
  startup_timer();
  ms5607_02ba_reset();
  ms5607_02ba_load_prom();

  int temp_debug_variable_breakpoint = 10;


  for(;;) {
	// Wait until a timed event occurs. This is when other tasks get to execute
    xQueueReceive(sensor_queue, &timer_update, portMAX_DELAY);

	// Operate on sensors based on what timed event occurred
    switch(timer_update.type) {
	  case SENSOR_CHECK_TWI_3200Hz:
		  // If the event was a 3.2khz tick, update the get data countdowns
		  if (get_ms5607_data_countdown >= 0)
		  {
			  get_ms5607_data_countdown -= 1;
			  if(get_ms5607_data_countdown == 0)
			  {
				  xQueueSendToBack(twie.twi_todo_queue, &ms5607_02ba.prepareADC, NULL);
				  if (altimeter_poll_mode == PRESSURE || altimeter_poll_mode == START_UP) {
					xQueueSendToBack(twie.twi_todo_queue, &ms5607_02ba.getADC_pressure, NULL);
				  }
				  else {
					  xQueueSendToBack(twie.twi_todo_queue, &ms5607_02ba.getADC_temperature, NULL);
				  }
			  }
		  }
			// Add more countdowns here
		break;
      case SENSOR_ENTRY_800Hz:
		temp_debug_variable_breakpoint += 1;
        break;
	  case SENSOR_ENTRY_400Hz:
		ms5607_02ba_convert_d2();
	  	get_ms5607_data_countdown = 26;
	    break;
      case SENSOR_ENTRY_100Hz:
		    temp_debug_variable_breakpoint += 3;
        //hmc5883l_read_data_single(&magnetometer_data);
        break;
	  case SENSOR_ENTRY_50Hz:
		  Si7021_A20_issue_rh_read_holds();
		  // Si7021_A20_issue_rh_read_noholds();
		  Si7021_A20_receive_rh_read();
      hmc5883l_read_data();
		  break;
	  case SENSOR_ENTRY_NONE:
	    break;
    }
    timer_update.type = SENSOR_ENTRY_NONE;

	ms5607_02ba_fetch_queue_press(&current_sensor_readings);
	ms5607_02ba_fetch_queue_temp(&current_sensor_readings);

    Si7021_A20_fetch_queue_rh(&current_sensor_readings);

    hmc5883l_fetch_queue_data(&current_sensor_readings);

	// send_to_tranceiver();
	twi_process_queue();
  }
}

static void sensor_initialize() {
	ms5607_02ba_init();
	// nrf24l01p_init();
    ms5607_02ba_reset();
    twi_process_queue();
    ms5607_02ba_load_prom();
    // hmc5883l_init();
    // hmc5883l_configure();
    // startup_timer();
}

// NOTE: send_to_transceiver not yet used
// ======================
//static void send_to_tranceiver() {
//	transceiver_message_t created_message;
//
//	// format message from struct based on message
//	created_message = transceiver_message_create(TRANSCEIVER_MSG_TYPE_GENERAL, &current_sensor_readings);
//
//	// transceiver_send_message/transceiver_message_create
//	transceiver_send_message(created_message, 0);
//}

// ================================================================================================
// ======================== Sensor loop timer handling ============================================
// ================================================================================================

static void startup_timer()
{
	tc_enable(&SENSOR_TIMER);
	tc_set_wgm(&SENSOR_TIMER, TC_WG_NORMAL);
	tc_write_period(&SENSOR_TIMER, F_CPU / (32 * BASE_HERTZ_MODIFIER));

	tc_set_overflow_interrupt_callback(&SENSOR_TIMER, protocol_timer_overflow_handler);
	tc_set_overflow_interrupt_level(&SENSOR_TIMER, TC_INT_LVL_LO);

	tc_write_clock_source(&SENSOR_TIMER, TC_CLKSEL_DIV1_gc);
}

static void protocol_timer_overflow_handler() {
	static uint8_t hertz_state = 0;
	static uint8_t fifty_hertz_state = 0;

	if (hertz_state == 0) {
		add_priority_event(SENSOR_ENTRY_100Hz, NULL);
		add_priority_event(SENSOR_ENTRY_400Hz, NULL);
		add_priority_event(SENSOR_ENTRY_800Hz, NULL);
		if (fifty_hertz_state == 0) {
			add_priority_event(SENSOR_ENTRY_50Hz,NULL);
		}
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

		if (fifty_hertz_state == 0)
			fifty_hertz_state+=1;
		else
			fifty_hertz_state = 0;
	}
	else{
		hertz_state+=1;
	}
}

// Is the one we need, or the one before this?
//static void protocol_timer_overflow_handler() {
	//static uint8_t hertz_state = 0;
//
	//if (hertz_state == 0) {
		//add_priority_event(SENSOR_ENTRY_100Hz, NULL);
	//}
	//else if (!(hertz_state & 1)) {
		//add_priority_event(SENSOR_ENTRY_400Hz, NULL);
	//}
	//add_priority_event(SENSOR_ENTRY_800Hz, NULL);
//
	//if (hertz_state == 7)
	//{
		//hertz_state = 0;
	//}
	//else{
		//hertz_state+=1;
	//}
//}

static inline void add_priority_event(enum sensor_queue_state_type event_type, void *data) {
	sensor_timer_t event = {
		.type = event_type
	};
	memcpy(event.data, data, EVENT_DATA_SIZE_MAX);
	xQueueSendToFront(sensor_queue, &event, portMAX_DELAY);
};

// ================================================================================================
// ================================================================================================
// ================================================================================================

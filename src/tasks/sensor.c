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

#include "ms5607_02ba.h"
// #include "fxls8471qr1.h"
// #include "l3g4200d.h"
#include "nrf24l01p.h"
//#include "data_buffer.h"
#include "message_types.h"
#include "transceiver.h"

/* Private Prototypes */
static void sensor_task_loop();
static void sensor_read();
static void sensor_initialize();
static void startup_timer();
static void send_to_tranceiver();

/* Private Variables */
#define SENSOR_COLLECTION_TIMER	TCC2
TaskHandle_t sensor_task_handle;
uint8_t sensor_read_cycle_counter=0;
sensors_message_t current_sensor_readings;
  
void sensor_start_task(void) {
  xTaskCreate(sensor_task_loop, "sensor", 1024, NULL, 2, &sensor_task_handle);
}


/* Private Functions */
static void sensor_task_loop() {
  TaskHandle_t sensor_task_handle;

  memset(&current_sensor_readings, 0, sizeof(sensors_message_t));
  
  sensor_initialize();
  startup_timer();
  
  for(;;) {
    sensor_read();
  }
}  

static void sensor_initialize() {
  ms5607_02ba_reset();
  ms5607_02ba_load_prom();
  
  // fxls8471qr1_init(void);
  // l3g4200d_init();
  
  nrf24l01p_init();
}

static void sensor_read() {
  uint8_t bool_reset_counter;
  
  if (sensor_read_cycle_counter = 0) {
	// read sensor at 400 Hz
	  
	// read sensor at 100 Hz
	  
  }
  else if((sensor_read_cycle_counter % 2) ==  1) {
    // read sensor at 400 Hz
  }
  
  // read sensor at 800 Hz
  
  if (sensor_read_cycle_counter==7) {
    sensor_read_cycle_counter = 0;
  }
  
  send_to_tranceiver(current_sensor_readings);
  startup_timer()
}

static void startup_timer()  {
  // set up timer
  tc_enable(&SENSOR_COLLECTION_TIMER);
  tc_set_overflow_interrupt_callback(&SENSOR_COLLECTION_TIMER, sensor_read);
  tc_set_wgm(&SENSOR_COLLECTION_TIMER, TC_WG_NORMAL);
  tc_write_period(&SENSOR_COLLECTION_TIMER, 1.25);
  tc_set_overflow_interrupt_level(&SENSOR_COLLECTION_TIMER, TC_INT_LVL_LO);
  cpu_irq_enable();
  tc_write_clock_source(&SENSOR_COLLECTION_TIMER, TC_CLKSEL_DIV1_gc);
}

static void send_to_tranceiver() {
  transceiver_message_t created_message;
	
  // format message from struct based on message
  created_message = transceiver_message_create(TRANSCEIVER_MSG_TYPE_GENERAL, &current_sensor_readings);
	 
  // transceiver_send_message/transceiver_message_create
  transceiver_send_message(created_message, 0);
}
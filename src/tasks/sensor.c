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
#include "data_buffer.h"

/* Private Prototypes */
static void sensor_task_loop();
static void sensor_read();
static void sensor_initialize();
static void startup_timer();

/* Private Variables */
#define SENSOR_COLLECTION_TIMER	TCC2
TaskHandle_t sensor_task_handle;

void sensor_start_task(void) {
  xTaskCreate(sensor_task_loop, "sensor", 1024, NULL, 2, &sensor_task_handle);
}


/* Private Functions */
static void sensor_task_loop() {
  TaskHandle_t sensor_task_handle;
  reference_node_t current_sensor_readings;
  transceiver_queue_t tranceiver_buffer;

  data_buffer_init(&tranceiver_buffer, &current_sensor_readings);
  
  sensor_initialize();
  startup_timer();
  
  for(;;) {
    
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
	
}

static void startup_timer()  {
  // set up timer
  tc_enable(&SENSOR_COLLECTION_TIMER);
  tc_set_overflow_interrupt_callback(&SENSOR_COLLECTION_TIMER, sensor_read);
  tc_set_wgm(&SENSOR_COLLECTION_TIMER, TC_WG_NORMAL);
  tc_write_period(&SENSOR_COLLECTION_TIMER, 1000);
  tc_set_overflow_interrupt_level(&SENSOR_COLLECTION_TIMER, TC_INT_LVL_LO);
  cpu_irq_enable();
  tc_write_clock_source(&SENSOR_COLLECTION_TIMER, TC_CLKSEL_DIV1_gc);
}
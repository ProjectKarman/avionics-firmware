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

/* Private Prototypes */
static void sensor_task_loop(void *p);

/* Private Variables */
TaskHandle_t sensor_task_handle;

void sensor_start_task(void) {
  xTaskCreate(sensor_task_loop, "sensor", 1024, NULL, 2, &sensor_task_handle);
}


/* Private Functions */
static void sensor_task_loop(void *p) {
  ms5607_02ba_reset();
  ms5607_02ba_load_prom();
  
  for(;;) {
    
  }
}  
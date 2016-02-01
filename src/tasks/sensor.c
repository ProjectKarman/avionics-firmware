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
#include <stdbool.h>

//#include "ms5607_02ba.h"
#include "l3g4200d.h"

/* Private Prototypes */
static void sensor_task_loop(void *p);

/* Private Variables */
TaskHandle_t sensor_task_handle;

void sensor_start_task(void) {
  xTaskCreate(sensor_task_loop, "sensor", 1024, NULL, 2, &sensor_task_handle);
}


/* Private Functions */
static void sensor_task_loop(void *p) {
  /*ms5607_02ba_reset();
  ms5607_02ba_load_prom();
  ms5607_02ba_convert_d1(OSR_2048);
  vTaskDelay(10);
  ms5607_02ba_read_adc_async(NULL);*/
  l3g4200d_check_comms();
  l3g4200d_enable_fifo(true);
  l3g4200d_set_fifo_mode(L3G4200D_FIFO_MODE);
  l3g4200d_set_datarate_and_bandwidth(L3G4200D_DR_800_BW_110);
  l3g4200d_activate();
  vTaskDelay(10);
  l3g4200d_raw_xyz_t data;
  l3g4200d_get_data(&data);
  volatile int16_t t = (int16_t)data.x;
  for(;;) {
    vTaskDelay(10);
  }
}  
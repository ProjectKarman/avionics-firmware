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
#include "fxls8471qr1.h"

#include "sensor_types.h"

#define ACCEL_FS_RANGE FXLS8471QR1_FS_RANGE_8G

/* Private Prototypes */
static void sensor_task_loop(void *p);
static void init_ms5607_02ba(void);
static void init_fxls8471qr1(void);
static accel_data_t convert_raw_accel_data(fxls8471qr1_raw_accel_t *raw_data);

/* Private Variables */
TaskHandle_t sensor_task_handle;

void sensor_start_task(void) {
  xTaskCreate(sensor_task_loop, "sensor", 1024, NULL, 2, &sensor_task_handle);
}


/* Private Functions */
static void sensor_task_loop(void *p) {
  init_ms5607_02ba();
  init_fxls8471qr1();
  
  vTaskDelay(5);
  fxls8471qr1_raw_accel_t raw_data;
  fxls8471qr1_get_data(&raw_data);
  volatile accel_data_t data = convert_raw_accel_data(&raw_data);
  
  ms5607_02ba_convert_d1(OSR_2048);
  vTaskDelay(10);
  ms5607_02ba_read_adc_async(NULL);
  for(;;) {
    vTaskDelay(10);
  }
}

static void init_ms5607_02ba(void) {
  ms5607_02ba_reset();
  ms5607_02ba_load_prom();
}

static void init_fxls8471qr1(void) {
  if(fxls8471qr1_check_comms()) {
    // An error occurred
    return;
  }
  fxls8471qr1_set_fs_range(ACCEL_FS_RANGE);
  //fxls8471qr1_set_fifo_mode(FXLS8471QR1_FIFO_CIRCULAR_BUFFER);
  //fxls8471qr1_set_data_rate(FXLS8471QR1_DR_200_HZ);
  fxls8471qr1_activate();
}

static accel_data_t convert_raw_accel_data(fxls8471qr1_raw_accel_t *raw_data) {
  accel_data_t data;
  data.x = fxls8471qr1_convert_raw_accel_to_scalar(raw_data->x, ACCEL_FS_RANGE);
  data.y = fxls8471qr1_convert_raw_accel_to_scalar(raw_data->y, ACCEL_FS_RANGE);
  data.z = fxls8471qr1_convert_raw_accel_to_scalar(raw_data->z, ACCEL_FS_RANGE);
  return data;
}
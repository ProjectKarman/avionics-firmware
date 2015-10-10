/*
 * transceiver.c
 *
 * Created: 10/8/2015 5:20:31 PM
 *  Author: Nigil Lee
 */ 

 #include "FreeRTOS.h"
 #include "task.h"
 #include "queue.h"

 #include "transceiver.h"
 #include "nrf24l01p.h"

 static QueueHandle_t transciver_send_queue;

static void init_nrf24l01p();

 void transceiver_task_loop(void *p) {
  init_nrf24l01p();

  // Setup queue


  //nrf24l01p_data_test();

  for(;;) {
    
  }
 }

 static void init_nrf24l01p() {
  nrf24l01p_read_regs();
  nrf24l01p_wake();
  vTaskDelay(3);
  nrf24l01p_flush_rx_fifo();
  nrf24l01p_flush_tx_fifo();
  nrf24l01p_set_channel(20);
  nrf24l01p_set_data_rate(NRF24L01P_DR_2M);
  nrf24l01p_set_pa_power(NRF24L01P_PWR_N18DBM);
 }
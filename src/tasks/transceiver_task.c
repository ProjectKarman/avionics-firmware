/*
 * transceiver_task.c
 *
 * Created: 10/8/2015 5:20:31 PM
 *  Author: Nigil Lee
 */ 

 #include "FreeRTOS.h"
 #include "task.h"

 #include "transceiver_task.h"
 #include "nrf24l01p.h"

 void transceiver_task_loop(void *p) {
  nrf24l01p_read_regs();
  nrf24l01p_wake();
  vTaskDelay(3);
  nrf24l01p_flush_rx_fifo();
  nrf24l01p_flush_tx_fifo();
  nrf24l01p_set_channel(20);
  nrf24l01p_set_data_rate(NRF24L01P_DR_2M);
  nrf24l01p_set_pa_power(NRF24L01P_PWR_N18DBM);
  nrf24l01p_data_test();

  for(;;) {
    
  }
 }
/*
 * nrf24l01p.h
 *
 * Created: 10/3/2015 11:55:24 AM
 *  Author: Nigil Lee
 */ 


#ifndef NRF24L01P_H_
#define NRF24L01P_H_

#include <inttypes.h>
#include <stddef.h>
#include "FreeRTOS.h"

// Driver Types
enum nrf24l01p_data_rate {
  NRF24L01P_DR_250K,
  NRF24L01P_DR_1M,
  NRF24L01P_DR_2M,
};

enum nrf24l01p_radio_mode {
  NRF24L01P_MODE_TX,
  NRF24L01P_MODE_RX,
};

enum nrf24l01p_pa_power {
  NRF24L01P_PWR_0DBM,
  NRF24L01P_PWR_N6DBM,
  NRF24L01P_PWR_N12DBM,
  NRF24L01P_PWR_N18DBM,
};

enum nrf24l01p_interrupt_mask {
  NRF24L01P_INTR_RX_DR = (1 << 0),
  NRF24L01P_INTR_TX_DS = (1 << 1),
  NRF24L01P_INTR_MAX_RT = (1 << 2),
};

typedef uint8_t nrf24l01p_interrupt_mask_t;

typedef void (*nrf24l01p_callback_t)(void);

// Driver Functions
void nrf24l01p_init(void);
void nrf24l01p_read_regs(void);
void nrf24l01p_set_data_rate(enum nrf24l01p_data_rate new_dr);
void nrf24l01p_set_radio_mode(enum nrf24l01p_radio_mode mode);
void nrf24l01p_set_pa_power(enum nrf24l01p_pa_power new_pwr);
void nrf24l01p_set_interrupt_mask(nrf24l01p_interrupt_mask_t mask);
void nrf24l01p_reset_interrupts(void);
void nrf24l01p_set_channel(uint8_t channel_num);
void nrf24l01p_open(void);
void nrf24l01p_wake(void);
void nrf24l01p_sleep(void);
uint8_t nrf24l01p_flush_tx_fifo(void);
uint8_t nrf24l01p_flush_rx_fifo(void);
void nrf24l01p_send_payload(uint8_t *data, size_t data_len);
void nrf24l01p_init_tx_payload_xfer(uint8_t *data, size_t data_len, nrf24l01p_callback_t xfer_complete_callback);
void nrf24l01p_set_interrupt_pin_handler(nrf24l01p_callback_t callback);
void nrf24l01p_read_register(uint8_t address, uint8_t *reg_value);
uint8_t nrf24l01p_write_register_single(uint8_t address, uint8_t new_value);
uint8_t nrf24l01p_write_register(uint8_t address, uint8_t const *new_value, uint8_t value_len);
uint8_t nrf24l01p_write_register_async(uint8_t address, uint8_t const *new_value, uint8_t value_len, nrf24l01p_callback_t callback);
void nrf24l01p_start_operation(void);
void nrf24l01p_end_operation(void);

#endif /* NRF24L01P_H_ */
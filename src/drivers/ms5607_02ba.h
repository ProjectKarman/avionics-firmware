/*
 * ms5607_02ba.h
 *
 * Created: 10/03/2015 11:55:36 PM
 *  Author: Timothy Rupprecht
 */ 

#include <stdint.h>
#include <string.h>

#ifndef MS5607_02BA_H_
#define MS5607_02BA_H_

void ms5607_02ba_init(void);
void ms5607_02ba_reset(void);
uint32_t ms5607_02ba_get_pressure(uint16_t);
uint32_t ms5607_02ba_get_tempurature(uint16_t);
uint32_t ms5607_02ba_read_adc(void);


void ms5607_02ba_wake();
void ms5607_02ba_flush_tx_fifo(void);
void ms5607_02ba_flush_rx_fifo(void);
void ms5607_02ba_write_register(uint8_t address, uint8_t new_value);
void ms5607_02ba_write_register_m(uint8_t address, uint8_t *new_value, size_t value_len);
void ms5607_02ba_send_payload(uint8_t *data, size_t data_len);
void ms5607_02ba_data_test(void);

#endif /* MS5607_02BA_H_ */
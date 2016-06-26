/*
 * uart_io.h
 *
 * Created: 11/19/2015 5:08:21 PM
 *  Author: Nigil Lee
 */ 


#ifndef UART_IO_H_
#define UART_IO_H_

#include <stdint.h>

void uart_io_init(void);
void uart_io_putchar(char *str_to_send, uint8_t len);

#endif /* UART_IO_H_ */
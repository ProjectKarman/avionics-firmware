/*
 * uart_io.c
 *
 * Created: 11/19/2015 5:08:10 PM
 *  Author: Nigil Lee
 */ 

#include "uart_io.h"

#include "asf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Defines */
#define UART_TX_QUEUE_DEPTH 20

// Peripheral Defines
#define CMD_USART USARTD0
#define USART_BSEL 12
#define USART_BSCALE 3

// IO Pins
#define USART_TX_PIN IOPORT_CREATE_PIN(PORTD, 3)
#define USART_RX_PIN IOPORT_CREATE_PIN(PORTD, 2)

/* Private Data Structures */
typedef struct {
  char *str;
  uint8_t len;
} string_data_t;

/* Private Variables */
static QueueHandle_t tx_queue;
static char *current_str;
static uint8_t current_str_len;
static uint8_t current_char_index;

void uart_io_init(void) {
  // Set USART baud rate to 19200 per datasheet
  CMD_USART.BAUDCTRLA = USART_BSEL;
  CMD_USART.BAUDCTRLB = USART_BSCALE << USART_BSCALE0_bp;
  
  // Set USART to 8 bit frame width
  CMD_USART.CTRLC = USART_CHSIZE1_bm | USART_CHSIZE0_bm;
  
  // Mode of operation is Async UART by default
  
  // Enable TX and RX
  CMD_USART.CTRLB |= USART_TXEN_bm | USART_RXEN_bm;
  
  // Setup Interrupts
  CMD_USART.CTRLA |= USART_RXCINTLVL_LO_gc;
  CMD_USART.CTRLA |= USART_TXCINTLVL_LO_gc;
  
  tx_queue = xQueueCreate(UART_TX_QUEUE_DEPTH, sizeof(string_data_t));
}

void uart_io_putchars(char *str_to_send, uint8_t len) {
  string_data_t str = {
    .str = str_to_send,
    .len = len
  }; 
  xQueueSendToBack(str, tx_queue, 0);
}

void uart_io_putchars_from_isr(char *str_to_send, uint8_t len) {
  string_data_t str = {
    .str = str_to_send,
    .len = len
  };
  xQueueSendToBackFromISR(str, tx_queue, NULL);
  current_str = str_to_send;
  current_str_len = len;
}

ISR(USARTD0_RXC_vect) {
  
}

ISR(USARTD0_TXC_vect) {
  
}
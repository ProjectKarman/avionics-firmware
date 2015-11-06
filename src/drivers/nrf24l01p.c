/*
 * nrf24l01p.c
 *
 * Created: 10/3/2015 11:54:58 AM
 *  Author: Nigil Lee
 */ 

#include "asf.h"
#include "usart_spi.h"
#include "nrf24l01p.h"
#include <util/delay.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

// Chip SPI Commands
#define SPICMD_R_REGISTER(reg) (0x00 | reg)
#define SPICMD_W_REGISTER(reg) (0x20 | reg)
#define SPICMD_R_RX_PAYLOAD 0x61
#define SPICMD_W_TX_PAYLOAD 0xA0
#define SPICMD_FLUSH_TX 0xE1
#define SPICMD_FLUSH_RX 0xE2
#define SPICMD_REUSE_TX_PL 0xE3

// Register addresses
#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_TX_ADDR 0x10
#define REG_FIFO_STATUS 0x17

// CONFIG reg bits
#define CONFIG_PRIM_RX (1 << 0)
#define CONFIG_PWR_UP (1 << 1)
#define CONFIG_MASK_MAX_RT (1 << 4)
#define CONFIG_MASK_TX_DS (1 << 5)
#define CONFIG_MASK_RX_DR (1 << 6)

// STATUS reg bits
#define STATUS_MAX_RT (1 << 4)
#define STATUS_TX_DS (1 << 5)
#define STATUS_RX_DR (1 << 6)

// RF SETUP reg bits
#define RF_SETUP_PLL_LOCK (1 << 4)
#define RF_SETUP_RF_DR_LOW (1 << 5)
#define RF_SETUP_RF_DR_HIGH (1 << 3)
#define RF_SETUP_RF_PWR1 (1 << 2)
#define RF_SETUP_RF_PWR0 (1 << 1)

// IO Definitions
#define SPI_MOSI_PIN IOPORT_CREATE_PIN(PORTC, 3)
#define SPI_MISO_PIN IOPORT_CREATE_PIN(PORTC, 2)
#define SPI_SCLK_PIN IOPORT_CREATE_PIN(PORTC, 1)
#define SPI_CS_PIN IOPORT_CREATE_PIN(PORTB, 6)
#define CE_PIN IOPORT_CREATE_PIN(PORTB, 7)
#define IRQ_PIN IOPORT_CREATE_PIN(PORTC, 0)

// Peripheral Definitions
#define SPI_CNTL_USART USARTC0
#define SPI_CNTL (&SPI_CNTL_USART)
#define DMA_CHANNEL 1

// Function Prototypes
static void spi_startframe(void);
static void spi_endframe(void);
static void dma_channel_handler(enum dma_channel_status status);

// Driver Variables
static uint8_t local_reg_config;
static uint8_t local_reg_rf_setup;
static nrf24l01p_callback_t dma_callback;
static nrf24l01p_callback_t interrupt_callback;
static nrf24l01p_callback_t current_function_callback;
static uint8_t *bytes_to_send;
static uint8_t bytes_len;
static uint8_t current_byte_index;
static SemaphoreHandle_t command_semaphore;
static SemaphoreHandle_t function_mutex;
static bool is_command_async;

void nrf24l01p_init(void) {
  // Init GPIOs
  ioport_set_pin_dir(SPI_MOSI_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(SPI_MISO_PIN, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(SPI_SCLK_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(SPI_CS_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(CE_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(IRQ_PIN, IOPORT_DIR_INPUT);

  ioport_set_pin_sense_mode(IRQ_PIN, IOPORT_SENSE_FALLING);
  arch_ioport_pin_to_base(IRQ_PIN)->INT0MASK |= arch_ioport_pin_to_mask(IRQ_PIN);
  arch_ioport_pin_to_base(IRQ_PIN)->INTCTRL |= PORT_INT0LVL1_bm;

  // Init SPI
  struct usart_spi_device spi_conf = {
    .id = SPI_CS_PIN
  };
  usart_spi_init(SPI_CNTL);
  usart_spi_setup_device(SPI_CNTL, &spi_conf, SPI_MODE_0, 8000000, 0);
  spi_endframe();

  // OS Level Structures
  function_mutex = xSemaphoreCreateMutex();
  command_semaphore = xSemaphoreCreateBinary();
}

void nrf24l01p_read_regs(void) {
  nrf24l01p_read_register(REG_CONFIG, &local_reg_config);
  nrf24l01p_read_register(REG_RF_SETUP, &local_reg_rf_setup);
}

void nrf24l01p_set_data_rate(enum nrf24l01p_data_rate new_dr) {
  switch(new_dr)
  {
    case NRF24L01P_DR_250K:
      local_reg_rf_setup |= RF_SETUP_RF_DR_LOW;
      local_reg_rf_setup &= ~RF_SETUP_RF_DR_HIGH;
      break;
    case NRF24L01P_DR_1M:
      local_reg_rf_setup &= ~RF_SETUP_RF_DR_LOW;
      local_reg_rf_setup &= ~RF_SETUP_RF_DR_HIGH;
      break;
    case NRF24L01P_DR_2M:
      local_reg_rf_setup &= ~RF_SETUP_RF_DR_LOW;
      local_reg_rf_setup |= RF_SETUP_RF_DR_HIGH;
      break;
  }
  nrf24l01p_write_register(REG_RF_SETUP, local_reg_rf_setup);
};

void nrf24l01p_set_radio_mode(enum nrf24l01p_radio_mode mode) {
  switch(mode)
  {
    case NRF24L01P_MODE_TX:
      local_reg_config &= ~CONFIG_PRIM_RX;
  	  break;
    case NRF24L01P_MODE_RX:
      local_reg_config |= CONFIG_PRIM_RX;
  	  break;
  }
  nrf24l01p_write_register(REG_CONFIG, local_reg_config);
};

void nrf24l01p_set_pa_power(enum nrf24l01p_pa_power new_pwr) {
  switch(new_pwr)
  {
    case NRF24L01P_PWR_0DBM:
      local_reg_rf_setup |= RF_SETUP_RF_PWR1 | RF_SETUP_RF_PWR0;
      break;
    case NRF24L01P_PWR_N6DBM:
      local_reg_rf_setup |= RF_SETUP_RF_PWR1;
      local_reg_rf_setup &= ~RF_SETUP_RF_PWR0;
      break;
    case NRF24L01P_PWR_N12DBM:
      local_reg_rf_setup &= ~RF_SETUP_RF_PWR1;
      local_reg_rf_setup |= RF_SETUP_RF_PWR0;
      break;
    case NRF24L01P_PWR_N18DBM:
    local_reg_rf_setup &= ~RF_SETUP_RF_PWR1;
    local_reg_rf_setup &= ~RF_SETUP_RF_PWR0;
    break;
  }
  nrf24l01p_write_register(REG_RF_SETUP, local_reg_rf_setup);
};

void nrf24l01p_set_interrupt_mask(nrf24l01p_interrupt_mask_t mask) {
  local_reg_config &= ~0x70; // Clear target bits
  if(mask & NRF24L01P_INTR_RX_DR) {
    local_reg_config |= CONFIG_MASK_RX_DR;
  }
  if(mask & NRF24L01P_INTR_TX_DS) {
    local_reg_config |= CONFIG_MASK_TX_DS;
  }
  if(mask & NRF24L01P_INTR_MAX_RT) {
    local_reg_config |= CONFIG_MASK_MAX_RT;
  }
  nrf24l01p_write_register(REG_CONFIG, local_reg_config);
}

void nrf24l01p_reset_interrupts(void) {
  nrf24l01p_write_register(REG_STATUS, STATUS_MAX_RT | STATUS_RX_DR | STATUS_TX_DS);
}

void nrf24l01p_set_channel(uint8_t channel_num) {
  nrf24l01p_write_register(REG_RF_CH, channel_num);
};

void nrf24l01p_open(void) {
  
}

void nrf24l01p_wake(void) {
  local_reg_config |= CONFIG_PWR_UP;
  nrf24l01p_write_register(REG_CONFIG, local_reg_config);
}

void nrf24l01p_sleep(void) {
  local_reg_config &= ~CONFIG_PWR_UP;
  nrf24l01p_write_register(REG_CONFIG, local_reg_config);
}

void nrf24l01p_flush_tx_fifo(void) {
  spi_startframe();
  usart_spi_write_single(SPI_CNTL, SPICMD_FLUSH_TX);
  spi_endframe();
}

void nrf24l01p_flush_rx_fifo(void) {
  spi_startframe();
  usart_spi_write_single(SPI_CNTL, SPICMD_FLUSH_RX);
  spi_endframe();
}

void nrf24l01p_send_payload(uint8_t *data, size_t data_len) {
  spi_startframe();
  usart_spi_write_single(SPI_CNTL, SPICMD_W_TX_PAYLOAD);
  usart_spi_write_packet(SPI_CNTL, data, data_len);
  spi_endframe();
}

void nrf24l01p_init_tx_payload_xfer(uint8_t *data, size_t data_len, nrf24l01p_callback_t xfer_complete_callback) {
  spi_startframe();

  struct dma_channel_config dmach_conf;
  memset(&dmach_conf, 0, sizeof(dmach_conf));
  dma_channel_set_burst_length(&dmach_conf, DMA_CH_BURSTLEN_1BYTE_gc);
  dma_channel_set_transfer_count(&dmach_conf, data_len);
  dma_channel_set_src_reload_mode(&dmach_conf, DMA_CH_SRCRELOAD_TRANSACTION_gc);
  dma_channel_set_dest_reload_mode(&dmach_conf, DMA_CH_DESTRELOAD_NONE_gc);
  dma_channel_set_src_dir_mode(&dmach_conf, DMA_CH_SRCDIR_INC_gc);
  dma_channel_set_dest_dir_mode(&dmach_conf, DMA_CH_DESTDIR_FIXED_gc);
  dma_channel_set_source_address(&dmach_conf, (uint16_t)(uintptr_t)data);
  dma_channel_set_destination_address(&dmach_conf, (uint16_t)(uintptr_t)&SPI_CNTL_USART.DATA);
  dma_channel_set_trigger_source(&dmach_conf, DMA_CH_TRIGSRC_USARTC0_DRE_gc);
  dma_channel_set_single_shot(&dmach_conf);
  dma_callback = xfer_complete_callback;
  dma_set_callback(DMA_CHANNEL, dma_channel_handler);
  dma_channel_set_interrupt_level(&dmach_conf, DMA_INT_LVL_MED);
  dma_channel_write_config(DMA_CHANNEL, &dmach_conf);

  usart_spi_write_single(SPI_CNTL, SPICMD_W_TX_PAYLOAD);
  dma_channel_enable(DMA_CHANNEL);
}

void nrf24l01p_set_interrupt_pin_handler(nrf24l01p_callback_t callback) {
  interrupt_callback = callback;
}

void nrf24l01p_read_register(uint8_t address, uint8_t *reg_value) {
  spi_startframe();
  usart_spi_write_single(SPI_CNTL, SPICMD_R_REGISTER(address));
  usart_spi_read_single(SPI_CNTL, reg_value);
  spi_endframe();
}

uint8_t nrf24l01p_write_register(uint8_t address, uint8_t *new_value, uint8_t value_len, TickType_t block_time) {
  if(xSemaphoreTake(function_mutex, block_time) == pdTRUE) {
    spi_startframe();
    usart_put(SPI_CNTL, SPICMD_W_REGISTER(address)); // Write out the address
    is_command_async = false;
    usart_set_tx_interrupt_level(SPI_CNTL, USART_INT_LVL_MED); // Enable Interrupt source
    xSemaphoreTake(command_semaphore, portMAX_DELAY); // Wait for command to complete
    xSemaphoreGive(function_mutex);
    usart_set_tx_interrupt_level(SPI_CNTL, USART_INT_LVL_OFF);
    // SPI Frame is ended in the USART ISR
    return 0;
  }
  else {
    return 1;
  }
}

void nrf24l01p_write_register_m(uint8_t address, const uint8_t *new_value, uint8_t value_len) {
  spi_startframe();
  usart_spi_write_single(SPI_CNTL, SPICMD_W_REGISTER(address));
  usart_spi_write_packet(SPI_CNTL, new_value, value_len);
  spi_endframe();
}

uint8_t nrf24l01p_write_register_async(uint8_t address, const uint8_t *new_value, uint8_t value_len, nrf24l01p_callback_t callback) {

  return 0;
}

void inline nrf24l01p_start_operation(void) {
  ioport_set_pin_level(CE_PIN, IOPORT_PIN_LEVEL_HIGH);
}

void inline nrf24l01p_end_operation(void) {
  ioport_set_pin_level(CE_PIN, IOPORT_PIN_LEVEL_LOW);
}

void nrf24l01p_data_test(void) {
  nrf24l01p_write_register(REG_EN_AA, 0x0);
  nrf24l01p_write_register(REG_SETUP_RETR, 0x0);
  uint8_t addr[] = {0xcc, 0xff, 0x00, 0xff, 0xcc};
  nrf24l01p_write_register_m(REG_TX_ADDR, addr, 5);
  while(1) {
    uint8_t test_data[32], i;
    
    uint8_t n = 0;
    for(i = 0; i < 32; i++) {
      test_data[i] = n++;
    }
    
    nrf24l01p_send_payload(test_data, 32);
    ioport_set_pin_level(CE_PIN, IOPORT_PIN_LEVEL_HIGH);
    _delay_us(30);
    ioport_set_pin_level(CE_PIN, IOPORT_PIN_LEVEL_LOW);
  }
}

static void spi_startframe(void) {
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_LOW);
}

static void spi_endframe() {
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_HIGH);
}

static void dma_channel_handler(enum dma_channel_status status) {
  spi_endframe();
  // Clear USART FIFO
  SPI_CNTL_USART.CTRLB &= ~USART_RXEN_bm;
  SPI_CNTL_USART.CTRLB |= USART_RXEN_bm;

  if(dma_callback) {
    dma_callback();
  }
};

ISR(PORTC_INT0_vect) {
  if(interrupt_callback) {
    interrupt_callback();
  }
}

ISR(USARTC0_TXC_vect) {
  if(current_byte_index < bytes_len) {
    usart_put(SPI_CNTL, bytes_to_send[current_byte_index++]);
  }
  else {
    // Transfer complete
    spi_endframe();
    if(is_command_async) {
      if(current_function_callback) {
        current_function_callback();
      }
    }
    else {
      xSemaphoreGiveFromISR(command_semaphore, NULL);
    }
  }
}
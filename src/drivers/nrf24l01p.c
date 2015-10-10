/*
 * nrf24l01p.c
 *
 * Created: 10/3/2015 11:54:58 AM
 *  Author: Nigil Lee
 */ 

#include "asf.h"
#include "usart_spi.h"
#include "nrf24l01p.h"
#include "usart_spi.h"
#include <util/delay.h>

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
#define REG_TX_ADDR 0x10

// CONFIG reg bits
#define CONFIG_PRIM_RX (1 << 0)
#define CONFIG_PWR_UP (1 << 1)

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

#define SPI_CNTL (&USARTC0)

// Function Prototypes
static void spi_startframe(void);
static void spi_endframe(void);

// Driver Variables
static uint8_t local_reg_config;
static uint8_t local_reg_rf_setup;

struct usart_spi_device conf = {
    .id = SPI_CS_PIN
};

void nrf24l01p_init(void) {
  // Init GPIOs
  ioport_set_pin_dir(SPI_MOSI_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(SPI_MISO_PIN, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(SPI_SCLK_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(SPI_CS_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(CE_PIN, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(IRQ_PIN, IOPORT_DIR_INPUT);

  // Init SPI
  usart_spi_init(SPI_CNTL);
  usart_spi_setup_device(SPI_CNTL, &conf, SPI_MODE_0, 8000000, 0);
  spi_endframe();
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

void nrf24l01p_init_tx_payload_xfer(uint8_t *data, size_t data_len, void (*xfer_complete_callback)()) {
  spi_startframe();
}

void nrf24l01p_read_register(uint8_t address, uint8_t *reg_value) {
  spi_startframe();
  usart_spi_write_single(SPI_CNTL, SPICMD_R_REGISTER(address));
  usart_spi_read_single(SPI_CNTL, reg_value);
  spi_endframe();
}

void nrf24l01p_write_register(uint8_t address, uint8_t new_value) {
  spi_startframe();
  usart_spi_write_single(SPI_CNTL, SPICMD_W_REGISTER(address));
  usart_spi_write_single(SPI_CNTL, new_value);
  spi_endframe();
}

void nrf24l01p_write_register_m(uint8_t address, const uint8_t *new_value, size_t value_len) {
  spi_startframe();
  usart_spi_write_single(SPI_CNTL, SPICMD_W_REGISTER(address));
  usart_spi_write_packet(SPI_CNTL, new_value, value_len);
  spi_endframe();
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
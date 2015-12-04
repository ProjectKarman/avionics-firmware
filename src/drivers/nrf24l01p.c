/*
 * nrf24l01p.c
 *
 * Created: 10/3/2015 11:54:58 AM
 *  Author: Nigil Lee
 */ 

#include "asf.h"
#include "nrf24l01p.h"
#include <string.h>
#include <inttypes.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

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

// STATUS reg bits
#define STATUS_MAX_RT (1 << 4)
#define STATUS_TX_DS (1 << 5)
#define STATUS_RX_DR (1 << 6)

// IO Definitions
#define SPI_MOSI_PIN IOPORT_CREATE_PIN(PORTC, 3)
#define SPI_MISO_PIN IOPORT_CREATE_PIN(PORTC, 2)
#define SPI_SCLK_PIN IOPORT_CREATE_PIN(PORTC, 1)
#define SPI_CS_PIN IOPORT_CREATE_PIN(PORTB, 6)
#define CE_PIN IOPORT_CREATE_PIN(PORTB, 7)
#define IRQ_PIN IOPORT_CREATE_PIN(PORTC, 0)

// USART Peripheral
#define SPI_CNTL USARTC0
#define F_PER 32000000
#define DESIRED_BITRATE 10000000
#define USART_BSEL 1 // (F_PER / (2 * DESIRED_BITRATE) - 1)

// Misc
#define OP_BUFFER_LEN 32
#define SEMAPHORE_BLOCK_TIME 0
#define DUMMY_DATA 0xff
#define RF_DR_LOW_MASK 0x2

enum command_type {
  CMD_TYPE_SYNC,
  CMD_TYPE_ASYNC,
};

// Function Prototypes
static void read_register_single(uint8_t address, uint8_t *reg_value);
static void read_register(uint8_t address, uint8_t *reg_value, uint8_t value_len);
static void write_register_single(uint8_t address, uint8_t new_value);
static void write_register_single_async(uint8_t address, uint8_t new_value, nrf24l01p_callback_t callback);
static void write_register(uint8_t address, uint8_t const *new_value, uint8_t value_len);
static void write_register_async(uint8_t address, uint8_t const *new_value, uint8_t value_len, nrf24l01p_callback_t callback);
static void send_command(uint8_t command, uint8_t const *data, uint8_t data_len);
static void send_command_async(uint8_t command, uint8_t const *data, uint8_t data_len, nrf24l01p_callback_t callback);
static void spi_startframe(void);
static void spi_endframe(void);

// Driver Variables
static union {
  struct {
    uint8_t prim_rx : 1;
    uint8_t pwr_up : 1;
    uint8_t crc0 : 1;
    uint8_t mask_max_rt : 1;
    uint8_t mask_tx_ds : 1;
    uint8_t mask_rx_dr : 1;
  };
  uint8_t raw;
} local_reg_config;
static union {
  struct {
    uint8_t pad0 : 1;
    uint8_t rf_pwr : 2;
    uint8_t rf_dr_high : 1;
    uint8_t rf_pll_lock : 1;
    uint8_t rf_dr_low : 1;
    uint8_t pad1 : 1;
    uint8_t cont_wave : 1;
  };
  uint8_t raw;
} local_reg_rf_setup;
static union {
  struct {
    uint8_t enaa_p0 : 1;
    uint8_t enaa_p1 : 1;
    uint8_t enaa_p2 : 1;
    uint8_t enaa_p3 : 1;
    uint8_t enaa_p4 : 1;
    uint8_t enaa_p5 : 1;
  };
  uint8_t raw;  
} local_reg_en_aa;
static union {
  struct {
    uint8_t arc : 4;
    uint8_t ard : 4;
  };
  uint8_t raw;
} local_reg_setup_retr;
static volatile union {
  struct {
    uint8_t tx_full : 1;
    uint8_t rx_p_no : 3;
    uint8_t max_rt : 1;
    uint8_t tx_ds: 1;
    uint8_t rx_dr: 1;
  };
  uint8_t raw;
} local_reg_status;
static nrf24l01p_callback_t interrupt_callback;
static nrf24l01p_callback_t current_function_callback;
static uint8_t op_buffer[OP_BUFFER_LEN];
static uint8_t op_len;
static volatile uint8_t op_buffer_index;
static SemaphoreHandle_t command_complete_semaphore;
static SemaphoreHandle_t command_running_semaphore;
static enum command_type current_command_type;

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
  arch_ioport_pin_to_base(IRQ_PIN)->INTCTRL |= PORT_INT0LVL_HI_gc;

  // Init SPI
  sysclk_enable_module(SYSCLK_PORT_C, PR_USART0_bm);
  SPI_CNTL.CTRLA |= USART_RXCINTLVL_MED_gc;
  SPI_CNTL.CTRLB |= USART_TXEN_bm | USART_RXEN_bm;
  SPI_CNTL.CTRLC |= USART_CMODE_MSPI_gc;
  SPI_CNTL.CTRLC &= ~(USART_CHSIZE2_bm | USART_CHSIZE1_bm); // Set SPI Phase / Data order
  SPI_CNTL.BAUDCTRLA = USART_BSEL;
  spi_endframe();

  // OS Level Structures
  command_running_semaphore = xSemaphoreCreateBinary();
  if(command_running_semaphore) {
    xSemaphoreGive(command_running_semaphore);
  }
  command_complete_semaphore = xSemaphoreCreateBinary();
}

uint8_t nrf24l01p_read_regs(void)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    read_register_single(REG_CONFIG, &local_reg_config.raw);
    read_register_single(REG_RF_SETUP, &local_reg_rf_setup.raw);
    read_register_single(REG_EN_AA, &local_reg_en_aa.raw);
    read_register_single(REG_SETUP_RETR, &local_reg_setup_retr.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_set_autoack_mask(uint8_t mask) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_reg_en_aa.raw = mask;
    write_register_single(REG_EN_AA, local_reg_en_aa.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_set_retransmission_delay(uint8_t delay) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_reg_setup_retr.ard = delay;
    write_register_single(REG_EN_AA, local_reg_en_aa.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_set_retransmission_count(uint8_t count) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_reg_setup_retr.arc = count;
    write_register_single(REG_EN_AA, local_reg_en_aa.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_set_data_rate(enum nrf24l01p_data_rate new_dr)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    if(new_dr & RF_DR_LOW_MASK) {
      local_reg_rf_setup.rf_dr_low = 0x1;
    }
    else {
      local_reg_rf_setup.rf_dr_low = 0;
      local_reg_rf_setup.rf_dr_high = new_dr & RF_DR_LOW_MASK;
    }
    write_register_single(REG_RF_SETUP, local_reg_rf_setup.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_set_radio_mode(enum nrf24l01p_radio_mode mode)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_reg_config.prim_rx = mode;
    write_register_single(REG_CONFIG, local_reg_config.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_set_pa_power(enum nrf24l01p_pa_power new_pwr)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_reg_rf_setup.rf_pwr= new_pwr;
    write_register_single(REG_RF_SETUP, local_reg_rf_setup.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_set_interrupt_mask(nrf24l01p_interrupt_mask_t mask)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_reg_config.raw &= ~0x70; // Clear target bits
    if(mask & NRF24L01P_INTR_RX_DR) {
      local_reg_config.mask_rx_dr = 0x1;
    }
    if(mask & NRF24L01P_INTR_TX_DS) {
      local_reg_config.mask_tx_ds = 0x1;
    }
    if(mask & NRF24L01P_INTR_MAX_RT) {
      local_reg_config.mask_max_rt = 0x1;
    }
    write_register_single(REG_CONFIG, local_reg_config.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_reset_interrupts(void)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    write_register_single(REG_STATUS, STATUS_MAX_RT | STATUS_RX_DR | STATUS_TX_DS);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_reset_interrupts_async(nrf24l01p_callback_t callback)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    const uint8_t value = STATUS_MAX_RT | STATUS_RX_DR | STATUS_TX_DS;
    write_register_async(REG_STATUS, &value, 1, callback);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_reset_interrupts_async_from_isr(nrf24l01p_callback_t callback)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    const uint8_t value = STATUS_MAX_RT | STATUS_RX_DR | STATUS_TX_DS;
    write_register_async(REG_STATUS, &value, 1, callback);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_set_channel(uint8_t channel_num)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    write_register_single(REG_RF_CH, channel_num);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
};

uint8_t nrf24l01p_set_address(uint8_t *address, uint8_t address_len)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    write_register(REG_TX_ADDR, address, address_len);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
};

uint8_t nrf24l01p_wake(void)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_reg_config.pwr_up = 0x1;
    write_register_single(REG_CONFIG, local_reg_config.raw);
    vTaskDelay(2); // Delay per datasheet
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_sleep(void)
{
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    local_reg_config.pwr_up = 0;
    write_register_single(REG_CONFIG, local_reg_config.raw);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_flush_tx_fifo(void) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    send_command(SPICMD_FLUSH_TX, NULL, 0);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_flush_rx_fifo(void) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    send_command(SPICMD_FLUSH_RX, NULL, 0);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_send_payload(uint8_t *data, size_t data_len) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    send_command(SPICMD_W_TX_PAYLOAD, data, data_len);
    xSemaphoreGive(command_running_semaphore);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_send_payload_async(uint8_t *data, uint8_t data_len, nrf24l01p_callback_t callback) {
  if(xSemaphoreTake(command_running_semaphore, SEMAPHORE_BLOCK_TIME) == pdTRUE) {
    send_command_async(SPICMD_W_TX_PAYLOAD, data, data_len, callback);

    return 0;
  }
  else {
    return 1;
  }
}

uint8_t nrf24l01p_send_payload_async_from_isr(uint8_t *data, uint8_t data_len, nrf24l01p_callback_t callback) {
  if(xSemaphoreTakeFromISR(command_running_semaphore, NULL) == pdTRUE) {
    send_command_async(SPICMD_W_TX_PAYLOAD, data, data_len, callback);
    
    return 0;
  }
  else {
    return 1;
  }
}

/*
 * The following function sync sends the reset command and then sends the next payload async then returns
 * It was implemented this way to meet the strict timing deadline of sending new payloads between interrupts
 */
uint8_t nrf24l01p_reset_interrupts_and_send_payload_from_isr(uint8_t *data, uint8_t data_len) {
  if(xSemaphoreTakeFromISR(command_running_semaphore, NULL) == pdTRUE) {
    SPI_CNTL.CTRLA &= USART_RXCINTLVL_gm; // Disable USART interrupt
    
    // Reset Interrupts
    spi_startframe();
    SPI_CNTL.DATA = SPICMD_W_REGISTER(REG_STATUS);
    while(!(SPI_CNTL.STATUS & USART_DREIF_bm));
    SPI_CNTL.DATA = STATUS_MAX_RT | STATUS_RX_DR | STATUS_TX_DS;
    while(!(SPI_CNTL.STATUS & USART_TXCIF_bm));
    spi_endframe();
    SPI_CNTL.STATUS |= USART_TXCIF_bm;
    
    // Send Payload
    spi_startframe();
    SPI_CNTL.DATA = SPICMD_W_TX_PAYLOAD;
    for(uint8_t i = 0; i < data_len; i++) {
      while(!(SPI_CNTL.STATUS & USART_DREIF_bm));
      SPI_CNTL.DATA = data[i];
    }
    while(!(SPI_CNTL.STATUS & USART_TXCIF_bm));
    spi_endframe();
    SPI_CNTL.STATUS |= USART_TXCIF_bm;
    
    SPI_CNTL.CTRLA |= USART_RXCINTLVL_MED_gc;
    
    xSemaphoreGive(command_running_semaphore);
    return 0;
  }
  else {
    return 1;
  }
}

void nrf24l01p_set_interrupt_pin_handler(nrf24l01p_callback_t callback) {
  interrupt_callback = callback;
}

void inline nrf24l01p_start_operation(void) {
  ioport_set_pin_level(CE_PIN, IOPORT_PIN_LEVEL_HIGH);
}

void inline nrf24l01p_end_operation(void) {
  ioport_set_pin_level(CE_PIN, IOPORT_PIN_LEVEL_LOW);
}

static void inline read_register_single(uint8_t address, uint8_t *reg_value) {
  read_register(address, reg_value, 1);
}

static void read_register(uint8_t address, uint8_t *reg_value, uint8_t value_len) {
  send_command(SPICMD_R_REGISTER(address), NULL, value_len);
}

static void inline write_register_single(uint8_t address, uint8_t new_value) {
  write_register(address, &new_value, 1);
}

static void inline write_register_single_async(uint8_t address, uint8_t new_value, nrf24l01p_callback_t callback) {
  write_register_async(address, &new_value, 1, callback);
}

static void inline write_register(uint8_t address, uint8_t const *new_value, uint8_t value_len) {
  send_command(SPICMD_W_REGISTER(address), new_value, value_len);
}

static void write_register_async(uint8_t address, uint8_t const *new_value, uint8_t value_len, nrf24l01p_callback_t callback) {
  send_command_async(SPICMD_W_REGISTER(address), new_value, value_len, callback);
}

static void send_command(uint8_t command, uint8_t const *data, uint8_t data_len) { 
  if(data) {
    memcpy(op_buffer, data, data_len);
  }
  else {
    memset(op_buffer, DUMMY_DATA, data_len);
  }
  
  current_command_type = CMD_TYPE_SYNC;
  op_buffer_index = 1;
  op_len = data_len;

  spi_startframe();
  // Fill TX FIFO
  SPI_CNTL.DATA = command;
  while(!(SPI_CNTL.STATUS & USART_DREIF_bm));
  SPI_CNTL.DATA = data[0];
  xSemaphoreTake(command_complete_semaphore, portMAX_DELAY); // Wait for command to complete
  // SPI Frame is ended in the USART ISR
}

static void send_command_async(uint8_t command, uint8_t const *data, uint8_t data_len, nrf24l01p_callback_t callback) {
  if(data) {
    memcpy(op_buffer, data, data_len);
  }
  else {
    memset(op_buffer, DUMMY_DATA, data_len);
  }
  
  current_command_type = CMD_TYPE_ASYNC;
  op_buffer_index = 1;
  op_len = data_len;

  spi_startframe();
  // Fill TX FIFO
  SPI_CNTL.DATA = command;
  while(!(SPI_CNTL.STATUS & USART_DREIF_bm));
  SPI_CNTL.DATA = data[0];
}

static void spi_startframe(void) {
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_LOW);
}

static void spi_endframe() {
  ioport_set_pin_level(SPI_CS_PIN, IOPORT_PIN_LEVEL_HIGH);
}

ISR(PORTC_INT0_vect) {
  if(interrupt_callback) {
    interrupt_callback();
  }
}

ISR(USARTC0_RXC_vect) {
  if(op_buffer_index == 1) {
    // First byte we receive is the status byte
    local_reg_status.raw = SPI_CNTL.DATA;
  }
  else {
    // If bytes_received isn't NULL then we should save data
    op_buffer[op_buffer_index - 2] = SPI_CNTL.DATA;
  }

  if(op_buffer_index < op_len - 1) {
    SPI_CNTL.DATA = op_buffer[op_buffer_index];
    op_buffer_index++;
  }
  else {
    // End of transfer
    spi_endframe();
    if(current_command_type == CMD_TYPE_SYNC) {
      xSemaphoreGiveFromISR(command_complete_semaphore, NULL);
    }
    else {
      xSemaphoreGiveFromISR(command_running_semaphore, NULL);
      if(current_function_callback) {
        current_function_callback();
      }
    }
  }
}
#include "MS5607_02BA.h"
#include "asf.h"

// pins
#define CSB     IOPORT_CREATE_PIN(PORTC, 4)
#define MISO     IOPORT_CREATE_PIN(PORTC, 7)
#define MOSI     IOPORT_CREATE_PIN(PORTC, 6)
#define SCLK    IOPORT_CREATE_PIN(PORTC, 5)

// commands
#define RESET             0x1E
#define CONVERTD1_256     0x40
#define CONVERTD1_512     0x42
#define CONVERTD1_1024    0x44
#define CONVERTD1_2048    0x46
#define CONVERTD1_4096    0x48
#define CONVERTD2_256     0x50
#define CONVERTD2_512     0x52
#define CONVERTD2_1024    0x54
#define CONVERTD2_2048    0x56
#define CONVERTD2_4096    0x58
#define ADC_READ          0x00
#define PROM(reg)  (0xA0 | (reg << 1)  )

// IO Definitions
#define SPI_CS_PIN IOPORT_CREATE_PIN(PORTD, 0)

struct usart_spi_device conf = {
  .id = SPI_CS_PIN
};

struct PROM {
  uint16_t manufacturer;
  uint16_t coefficient_1;
  uint16_t coefficient_2;
  uint16_t coefficient_3;
  uint16_t coefficient_4;
  uint16_t coefficient_5;
  uint16_t coefficient_6;
  uint16_t crc;
};
typedef struct PROM PROM;

void ms5607_02ba_read_prom(void);
uint32_t ms5607_02ba_send_command(uint8_t);
uint32_t adjust_tempurature(uint32_t);
uint32_t adjust_pressure(uint32_t);


void ms5607_02ba_init() {
  
  ioport_set_pin_dir(CSB, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(MISO, IOPORT_DIR_INPUT);
  ioport_set_pin_dir(MOSI, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(SCLK, IOPORT_DIR_OUTPUT);

  ioport_set_pin_level(CSB, false);

  usart_spi_init(&SPID);
  usart_spi_setup_device(&SPID, &conf, SPI_MODE_0, 1000000, 0);
  // usart_spi_enable(&SPID);
  
  ms5607_02ba_reset();
  ms5607_02ba_read_prom();
  
  return;
 }

void ms5607_02ba_reset(void) {
  ms5607_02ba_send_command(RESET);
  return;
}

uint32_t ms5607_02ba_get_pressure(uint16_t resolution) {
  uint32_t raw_pressure = 0x00000;
  switch(resolution) {
    case 4096: raw_pressure = ms5607_02ba_send_command(CONVERTD1_4096); break;
    case 2048: raw_pressure = ms5607_02ba_send_command(CONVERTD1_2048); break;
    case 1024: raw_pressure = ms5607_02ba_send_command(CONVERTD1_1024); break;
    case 512: raw_pressure = ms5607_02ba_send_command(CONVERTD1_512); break;
    case 256: raw_pressure = ms5607_02ba_send_command(CONVERTD1_256); break;
    default: raw_pressure = ms5607_02ba_send_command(CONVERTD1_4096); break;
  }
  return adjust_pressure(raw_pressure);
}

uint32_t ms5607_02ba_get_tempurature(uint16_t resolution) {
  uint32_t raw_temp = 0x00000;
  switch(resolution) {
    case 4096: raw_temp = ms5607_02ba_send_command(CONVERTD2_4096); break;
    case 2048: raw_temp = ms5607_02ba_send_command(CONVERTD2_2048); break;
    case 1024: raw_temp = ms5607_02ba_send_command(CONVERTD2_1024); break;
    case 512: raw_temp = ms5607_02ba_send_command(CONVERTD2_512); break;
    case 256: raw_temp = ms5607_02ba_send_command(CONVERTD2_256); break;
    default: raw_temp = ms5607_02ba_send_command(CONVERTD2_4096); break;
  }
  return adjust_tempurature(raw_temp); 
}
uint32_t ms5607_02ba_read_adc(void) {
  uint32_t adc_value = 0x00000;
  adc_value = ms5607_02ba_send_command(ADC_READ);
  return adc_value;
}

// Private/Utility Functions

void ms5607_02ba_read_prom(void) {
  
  uint8_t data_buffer[2];
  PROM prom;
  
  prom.manufacturer = 0x0000;
  prom.coefficient_1 = 0x0000;
  prom.coefficient_2 = 0x0000;
  prom.coefficient_3 = 0x0000;
  prom.coefficient_4 = 0x0000;
  prom.coefficient_5 = 0x0000;
  prom.coefficient_6 = 0x0000;
  prom.crc = 0x0000;
  
  for (int prom_addr = 0; prom_addr < 16; prom_addr += 2) {
    usart_spi_select_device(&SPID, &conf);
    usart_spi_write_packet(&SPID, PROM(prom_addr), 1);
    usart_spi_read_packet(&SPID, data_buffer, 2);
    usart_spi_deselect_device(&SPID, &conf);
    
    switch(prom_addr) {
      case 0: prom.manufacturer = data_buffer; break;
      case 2: prom.coefficient_1 = data_buffer; break;
      case 4: prom.coefficient_2 = data_buffer; break;
      case 6: prom.coefficient_3 = data_buffer; break;
      case 8: prom.coefficient_4 = data_buffer; break;
      case 10: prom.coefficient_5 = data_buffer; break;
      case 12: prom.coefficient_6 = data_buffer; break;
      case 14: prom.crc = data_buffer; break;
      default: break;
    }
        
    data_buffer[0] = 0x00;
    data_buffer[1] = 0x00;
  }    
  
  return;
}

uint32_t ms5607_02ba_send_command(uint8_t comm) {
  
  uint8_t data_buffer[4];
  // struct spi_device spi_device_conf = {
  //  .id = IOPORT_CREATE_PIN(PORTD, 0)
  // };
  
  usart_spi_select_device(&SPID, &conf);
  usart_spi_write_packet(&SPID, comm, 1);
  usart_spi_read_packet(&SPID, data_buffer, 1);
  usart_spi_deselect_device(&SPID, &conf);
  
  return data_buffer;
}

uint32_t adjust_pressure(uint32_t raw_pressure) {
  uint32_t actual = 0x00000;
  return actual;
}

uint32_t adjust_tempurature(uint32_t raw_tempurature) {
  uint32_t actual = 0x00000;
  return actual;
}
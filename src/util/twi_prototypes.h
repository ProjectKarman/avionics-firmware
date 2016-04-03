#ifdef TWI_PROTOTYPES
#define TWI_PROTOTYPES

typedef void (*twi_isr_callback_t)(void);

typedef struct twi_t {
	uint8_t twi_device_addr;
	
	// write data members
	uint32_t twi_write_data;
	uint8_t twi_write_data_length;
	uint8_t twi_write_data_index;
	
	// read data members
	uint32_t twi_read_data;
	uint8_t twi_read_length;
	uint8_t twi_read_index;
	
	// boolean is i2c bus in use?
	uint8_t twi_bus_locked;
	
	struct twi_write {
		twi_isr_callback_t twi_write_isr_callback;
	};
	
	struct twi_read {
		twi_isr_callback_t twi_read_isr_callback;
	};
	
} twi_interface;

#endif
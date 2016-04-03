#ifdef TWI_PROTOTYPES
#define TWI_PROTOTYPES

#include "queue.h"

typedef void (*twi_isr_callback_t)(void);


typedef enum twi_task_mode {
	TWI_READ_MODE,
	TWI_WRITE_MODE,
	TWI_IDLE_MODE
} twi_todo_item_mode_t;

typedef struct twi_interface {
	uint8_t twi_device_addr;

	// define todo queue
	QueueHandle_t twi_todo_queue
	
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
	
} twi_interface_t;

typedef struct twi_task {
    QueueHandle_t* return_queue;
    uint8_t device_addr;
    twi_task_mode_t mode;
    uint8_t* write_data;
    unint8_t length;
} twi_todo_t;

uint8_t twi_init(); // Create queue and config registers
uint8_t twi_add_task_to_queue(twi_task_t* task);
uint8_t twi_process_queue();

uint8_t twi_write(uint8 device, uint8* data, unit8 length);
uint8_t twi_read(uint8 device, unit8 length);

// REMEMBER TO DEFINE ISR IN C FILE

#endif
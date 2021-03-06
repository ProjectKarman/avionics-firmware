#ifndef TWI_PROTOTYPES
#define TWI_PROTOTYPES

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "asf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "twi_interface.h"

// TWI Controller Settings
#define CTRLC_START 0x1
#define TWI_STATUS_IDLE (0x1 << 0)
#define TWI_SMART_MODE_EN (1 << 0)
#define TWI_QUICK_COMD_EN (1 << 1)

#define MAX_SENSOR_DATA_WRITE_LENGTH 10
#define MAX_SENSOR_DATA_READ_LENGTH 10

typedef void (*twi_isr_callback_t)(void);


typedef enum twi_task_mode
{
    TWI_READ_MODE,
    TWI_WRITE_MODE,
    TWI_IDLE_MODE
} twi_task_mode_t;

typedef struct twi_interface
{
    uint8_t twi_device_addr;

    // define todo queue
    QueueHandle_t twi_todo_queue;

    // write data members
    uint8_t twi_write_data_index;

    // read data members
    uint8_t twi_read_data[MAX_SENSOR_DATA_READ_LENGTH];
    uint8_t twi_read_index;

    // boolean is i2c bus in use?
    uint8_t twi_bus_locked;

} twi_interface_t;

typedef struct twi_task
{
    QueueHandle_t *return_queue;
    uint8_t device_addr;
    twi_task_mode_t mode;
    uint8_t write_data[MAX_SENSOR_DATA_WRITE_LENGTH];
    uint8_t length;
} twi_task_t;

uint8_t twi_init(void); // Create queue and config registers
uint8_t twi_add_task_to_queue(twi_task_t *task);

uint8_t start_task_from_queue(void);

uint8_t twi_process_queue(void);

uint8_t twi_process_queue_blocking(void);

uint8_t twi_write(uint8_t device, uint8_t *data, uint8_t length);

uint8_t twi_read(uint8_t device, uint8_t length);

// REMEMBER TO DEFINE ISR IN C FILE

#endif
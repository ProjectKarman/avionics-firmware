//
// Created by Bryce Carter &
//   		  Tim Rupprecht
// on 4/3/16.
//

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "asf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "twi_interface.h"

// TWI Peripheral Config
#define TWI_MASTER       TWIE
#define TWI_MASTER_PORT  PORTE
#define TWI_FREQ        400000
#define TWI_BAUD_REG 35 // ((F_SYS / (2 * TWI_FREQ)) - 5)

#define TWI_TODO_QUEUE_LENGTH 10

twi_interface_t twie;

uint8_t twi_init()
{
    // Set the hardware configuration
    // ===========================================================
    TWI_MASTER_PORT.PIN0CTRL |= PORT_OPC_WIREDANDPULL_gc;
    TWI_MASTER_PORT.PIN1CTRL |= PORT_OPC_WIREDANDPULL_gc;

    PR.PRPE &= ~PR_TWI_bm; // Enabled TWI module clock

    TWI_MASTER.MASTER.BAUD = TWI_BAUD_REG;
    TWI_MASTER.MASTER.CTRLA |= TWI_MASTER_ENABLE_bm | TWI_MASTER_WIEN_bm | TWI_MASTER_INTLVL_MED_gc;
    // ===========================================================

    // Set up higher level objects
    twie.twi_todo_queue = xQueueCreate(TWI_TODO_QUEUE_LENGTH, sizeof(twi_task_t));

    // Set the twie object properties
    twie.twi_bus_locked = 0;
    return 0;
}

uint8_t twi_add_task_to_queue(twi_task_t *task)
{
    xQueueSendToFront(twie.twi_todo_queue, task, portMAX_DELAY);
    return 1;
}

uint8_t start_task_from_queue(void)
{
    twi_task_t *task = 0;

    if (xQueuePeek(twie.twi_todo_queue, task, 0) == pdTRUE)
    {
        twie.twi_device_addr = task->device_addr;

        switch (task->mode)
        {
            case TWI_READ_MODE:
                twie.twi_read_index = 0;
                break;
            case TWI_WRITE_MODE:
                twie.twi_write_data_index = 0;
                break;
            case TWI_IDLE_MODE:
                return 0;
        }
        twie.twi_bus_locked = 1;
        TWI_MASTER.MASTER.ADDR = task->device_addr << 1;
        return 1; // item was processed from queue
    }
    return 2; // No items in queue to process
}


uint8_t twi_process_queue()
{
    if (twie.twi_bus_locked == 1)
    {
        return 3; // TWI Bus is not available
    }

    return start_task_from_queue();
}

uint8_t twi_process_queue_blocking()
{
    while (uxQueueMessagesWaiting(twie.twi_todo_queue) != 0)
    {
        if (twie.twi_bus_locked == 0)
        {
            start_task_from_queue();
        }
        vTaskDelay(1);
    }
    return 0;
}


ISR(TWIE_TWIM_vect)
{
    uint8_t task_done = 0;
    twi_task_t *current_task = 0;
    xQueuePeekFromISR(twie.twi_todo_queue, current_task);
    switch (current_task->mode)
    {
        case TWI_WRITE_MODE:
            if (twie.twi_write_data_index < current_task->length)
            {
                TWI_MASTER.MASTER.DATA = current_task->write_data[twie.twi_write_data_index];
                twie.twi_write_data_index += 1;
            }
            else
            {
                task_done = 1;
            }
            break;
        case TWI_READ_MODE:
            if (twie.twi_read_index < current_task->length)
            {
                twie.twi_read_data[twie.twi_read_index] = TWI_MASTER.MASTER.DATA;
                twie.twi_read_index += 1;
            }
            else
            {
                task_done = 1;
                int i = 0;
                for (; i < current_task->length; i++)
                {
                    xQueueSendToBackFromISR(current_task->return_queue, &twie.twi_read_data[i], NULL); // TODO: confirm that the pass by ref does not cause a problem here
                }
            }
        case TWI_IDLE_MODE:
            task_done = 1;
            break;
    }
    if (task_done)
    {
        TWI_MASTER.MASTER.CTRLC |= TWI_MASTER_CMD_STOP_gc; // Send stop command
        xQueueReceiveFromISR(twie.twi_todo_queue, current_task, NULL);
        twie.twi_bus_locked = 0;
    }
}




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

#define TWI_TODO_QUEUE_LENGTH

static twi_interface_t twie;

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
    twie.twi_todo_queue = xQueueCreate(TWI_TODO_QUEUE_LENGTH, sizeof(uint32_t));
}

uint8_t twi_add_task_to_queue(twi_task_t* task)
{
	xQueueSendToFront(twie.twi_todo_queue, task, NULL);
}



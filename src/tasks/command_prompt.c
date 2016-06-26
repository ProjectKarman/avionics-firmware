/*
 * command_prompt.c
 *
 * Created: 11/16/2015 11:58:12 AM
 *  Author: Nigil Lee
 */ 

#include <stdint.h>

#include "command_prompt.h"
#include "uart_io.h"

#include "asf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Defines */
#define PROMPT_EVENT_QUEUE_DEPTH 20


/* Private Data Structures */
enum prompt_event_type {
  PROMPT_EVENT_BYTE_RX,
  PROMPT_EVENT_BYTE_TX,
};

typedef struct {
  enum prompt_event_type type;
  uint8_t data;
} prompt_event_t;

/* Private Variables */
static TaskHandle_t command_prompt_task_handle;
static QueueHandle_t prompt_event_queue;

/* Private Function Prototypes */
static void command_prompt_task_loop(void *p);
static void process_rx(void);
static void process_tx(void);

/* Public Functions */
void command_prompt_start_task(void) {
  xTaskCreate(command_prompt_task_loop, "command_prompt", 512, NULL, 2, &command_prompt_task_handle);
  prompt_event_queue = xQueueCreate(PROMPT_EVENT_QUEUE_DEPTH, sizeof(prompt_event_t));
}

/* Private Functions */
static void command_prompt_task_loop(void *p) {
  
  
  prompt_event_t current_event;
  for(;;) {  
      //xQueueReceive(event_queue, &currentEvent, portMAX_DELAY);
      
      switch(current_event.type) {
        case PROMPT_EVENT_BYTE_RX:
          process_rx();
          break;
         case PROMPT_EVENT_BYTE_TX:
          process_tx();
          break;
      }
  }
}

static void process_rx(void) {
}

static void process_tx(void) {
  
}  
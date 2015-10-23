/*
 * message_types.c
 *
 * Created: 10/23/2015 10:35:13 AM
 *  Author: Nigil Lee
 */ 

#include <string.h>
#include "FreeRTOS.h"
#include "message_types.h"

general_message_t *general_message_create(void)
{
  general_message_t *message = (general_message_t *)pvPortMalloc(sizeof(general_message_t));
  if(message != NULL) {
    memset(message, 0, sizeof(general_message_t));
  }
  return message;
}

void general_message_destory(general_message_t *message) {
  vPortFree(message);
}
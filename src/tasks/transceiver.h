/*
 * transceiver.h
 *
 * Created: 10/8/2015 5:20:44 PM
 *  Author: Nigil Lee
 */ 


#ifndef TRANSCEIVER_H_
#define TRANSCEIVER_H_

#include "message_types.h"

enum transceiver_message_type {
  TRANSCEIVER_MSG_TYPE_GENERAL,
  TRANSCEIVER_MSG_TYPE_SENSORS
};

typedef struct {
  enum transceiver_message_type type;
  uint8_t data[MESSAGE_MAX_SIZE];
} transceiver_message_t;

extern TaskHandle_t transceiver_task_handle;

void transceiver_start_task(void);
void transceiver_send_message(transceiver_message_t message, TickType_t ticks_to_wait);
transceiver_message_t transceiver_message_create(enum transceiver_message_type type, void *contents);

#endif /* TRANSCEIVER_H_ */
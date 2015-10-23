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
  TRANSCEIVER_MSG_TYPE_GENERAL
};

typedef struct {
  enum transceiver_message_type type;
  void *data;
} transceiver_message_t;

extern TaskHandle_t transceiver_task_handle;

void transceiver_start_task(void);
void transceiver_send_message(transceiver_message_t *message, TickType_t ticks_to_wait);
transceiver_message_t *transceiver_message_create(void);
void transceiver_message_destroy(transceiver_message_t *message);

#endif /* TRANSCEIVER_H_ */
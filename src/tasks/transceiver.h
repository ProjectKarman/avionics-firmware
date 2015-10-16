/*
 * transceiver.h
 *
 * Created: 10/8/2015 5:20:44 PM
 *  Author: Nigil Lee
 */ 


#ifndef TRANSCEIVER_H_
#define TRANSCEIVER_H_

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

#endif /* TRANSCEIVER_H_ */
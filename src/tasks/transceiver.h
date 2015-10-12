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

struct transceiver_message {
  enum transceiver_message_type type;
  void *data;
};

enum transeiver_state {
  TRANSCEIVER_STATUS_IDLE,
  TRANSCEIVER_STATUS_SLEEP,
  TRANSCEIVER_STATUS_TRANSMITTING,
  TRANSCEIVER_STATUS_RECEIVING,
};

extern QueueHandle_t transceiver_send_queue;
extern TaskHandle_t transceiver_task_handle;

void transceiver_start_task(void);

#endif /* TRANSCEIVER_H_ */
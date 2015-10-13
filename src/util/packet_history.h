/*
 * packet_history.h
 *
 * Created: 10/12/2015 5:48:30 PM
 *  Author: Nigil Lee
 */ 


#ifndef PACKET_HISTORY_H_
#define PACKET_HISTORY_H_

#include <stdint.h>
#include <stddef.h>

#define PACKET_HISTORY_DEPTH 8

struct protocol_packet {
  uint8_t *bytes;
  size_t len;
  uint8_t index;
};

struct protocol_raw_frame {
  struct protocol_packet *packets;
  size_t len;
};

typedef struct {
  uint8_t current_size;
  struct protocol_raw_frame *history[PACKET_HISTORY_DEPTH];
} packet_history_t;

void packet_history_init(packet_history_t *history);
void packet_history_add(packet_history_t *history, struct protocol_raw_frame *frame);

#endif /* PACKET_HISTORY_H_ */
/*
 * packet_history.h
 *
 * Created: 10/12/2015 5:48:30 PM
 *  Author: Nigil Lee
 */ 


#ifndef PACKET_HISTORY_H_
#define PACKET_HISTORY_H_

#define PACKET_HISTORY_DEPTH

struct protocol_packet {
  uint8_t *bytes;
  size_t len;
  uint8_t index;
};

struct protocol_raw_frame {
  protocol_packet *packets;
  size_t len;
};

typedef struct {
  protocol_raw_frame history[PACKET_HISTORY_DEPTH];
  uint8_t current_size;
} packet_history_t;

void packet_history_init(packet_history_t *history)

#endif /* PACKET_HISTORY_H_ */
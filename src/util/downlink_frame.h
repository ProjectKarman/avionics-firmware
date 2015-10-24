/*
 * downlink_frame.h
 *
 * Created: 10/16/2015 4:06:53 PM
 *  Author: Nigil Lee
 */ 


#ifndef DOWNLINK_FRAME_H_
#define DOWNLINK_FRAME_H_

#include <stdint.h>

typedef struct downlink_packet {
  uint8_t *bytes;
  size_t len;
  struct downlink_packet *next_packet;
} downlink_packet_t;

typedef struct {
  downlink_packet_t *header_packet;
  downlink_packet_t *retransmit_section;
  downlink_packet_t *data_section;
  downlink_packet_t *data_section_tail;
  downlink_packet_t *tail_packet;
  downlink_packet_t *packet_prt;
} downlink_frame_t;

downlink_packet_t *downlink_packet_create(void);
void downlink_packet_destroy(downlink_packet_t *packet);
downlink_frame_t *downlink_frame_create(void);
void downlink_frame_destory(downlink_frame_t *frame);
void downlink_frame_add_packet(downlink_frame_t *frame, downlink_packet_t *packet);
void downlink_frame_copy_retransmissions(downlink_frame_t *frame, downlink_packet_t *retransmissions_head);
void downlink_frame_prepare_for_sending(downlink_frame_t *frame);
downlink_packet_t *downlink_frame_get_packet(downlink_frame_t *frame);


#endif /* DOWNLINK_FRAME_H_ */
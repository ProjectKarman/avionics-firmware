/*
 * downlink_frame.h
 *
 * Created: 10/16/2015 4:06:53 PM
 *  Author: Nigil Lee
 */ 


#ifndef DOWNLINK_FRAME_H_
#define DOWNLINK_FRAME_H_

#include <stdint.h>

#define PACKET_BYTE_LEN 33 // One extra byte for the DMA command if used
#define FRAME_MAX_PACKETS 12
#define FRAME_MAX_SIZE FRAME_MAX_PACKETS + 1 // One packet for the header

typedef struct downlink_packet {
  uint8_t bytes[PACKET_BYTE_LEN];
  uint8_t len;
} downlink_packet_t;

typedef struct {
  downlink_packet_t packets[FRAME_MAX_SIZE];
  downlink_packet_t *write_ptr;
  downlink_packet_t *read_ptr;
  uint8_t size;
} downlink_frame_t;

void downlink_frame_init(downlink_frame_t *frame);
void downlink_frame_add_packet(downlink_frame_t *frame, downlink_packet_t *packet);
void downlink_frame_write_header(downlink_frame_t *frame);
uint8_t downlink_frame_get_next_packet(downlink_frame_t *frame, downlink_packet_t *next_packet);

#endif /* DOWNLINK_FRAME_H_ */
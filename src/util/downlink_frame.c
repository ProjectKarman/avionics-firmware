/*
 * downlink_frame.c
 *
 * Created: 10/16/2015 4:04:29 PM
 *  Author: Nigil Lee
 */ 

#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "downlink_frame.h"

static uint16_t frame_count = 0;

void downlink_frame_init(downlink_frame_t *frame) {
  frame->read_ptr = frame->packets;
  frame->write_ptr = frame->packets + 1;
  frame->size = 1;
}

void downlink_frame_add_packet(downlink_frame_t *frame, downlink_packet_t *packet) {
  frame->size++;
  *(frame->write_ptr) = *packet;
  if(frame->write_ptr < frame->packets + FRAME_MAX_SIZE - 1 ) {
    frame->write_ptr++;
  }
  else {
    frame->write_ptr = frame->packets; 
  }
}

void downlink_frame_write_header(downlink_frame_t *frame) {
  downlink_packet_t header_packet;
  header_packet.bytes[1] = (frame_count >> 0*8 ) & 0xff;
  header_packet.bytes[2] = (frame_count >> 1*8 ) & 0xff;
  header_packet.bytes[3] = frame->size;
  header_packet.len = 3;
  frame->packets[0] = header_packet;
  frame_count++;
}

uint8_t downlink_frame_get_next_packet(downlink_frame_t *frame, downlink_packet_t *next_packet) {
  if(frame->size == 0) {
    return 1;
  }
  else {
    *next_packet = *(frame->read_ptr);
    if(frame->read_ptr < frame->packets + FRAME_MAX_SIZE - 1 ) {
      frame->read_ptr++;
    }
    else {
      frame->read_ptr = frame->packets;
    }
    frame->size--;
    return 0;
  }
}
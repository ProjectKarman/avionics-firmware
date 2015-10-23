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

downlink_packet_t *downlink_packet_create(void)
{
  downlink_packet_t *packet = (downlink_packet_t *)pvPortMalloc(sizeof(downlink_packet_t));
  memset(packet, 0, sizeof(downlink_packet_t));
  return packet;
}

void downlink_packet_destroy(downlink_packet_t *packet) {
  vPortFree(packet);
}

downlink_frame_t *downlink_frame_create(void)
{
  downlink_frame_t *frame = (downlink_frame_t *)pvPortMalloc(sizeof(downlink_frame_t));
  memset(frame, 0, sizeof(downlink_frame_t));
  return frame;
}

void downlink_frame_destory(downlink_frame_t *frame) {
  
  vPortFree(frame);
}

void downlink_frame_add_packet(downlink_frame_t *frame, downlink_packet_t *packet) {
  if(frame->data_section_tail == NULL) {
    frame->data_section = frame->data_section_tail = packet;
  }
  else {
    frame->data_section_tail->next_packet = packet;
    frame->data_section_tail = frame->data_section_tail;
  }
}

void downlink_frame_copy_retransmissions(downlink_frame_t *frame, downlink_packet_t *retransmissions_head) {
  frame->retransmit_section = retransmissions_head;
}

void downlink_frame_prepare_for_sending(downlink_frame_t *frame) {
  // Generate Header
  downlink_packet_t *header_packet = downlink_packet_create();
  header_packet->len = 1;
  header_packet->bytes = pvPortMalloc(sizeof(uint8_t));
  frame->header_packet = header_packet;

  downlink_packet_t *tail_packet = header_packet;
  uint8_t packet_index = 0;

  if(frame->retransmit_section != NULL) {
    // Link to retransmissions
    tail_packet->next_packet = frame->retransmit_section;  
    tail_packet = tail_packet->next_packet;
    
    // Iterate retransmission section, find tail and add indexes
    for(;;) {
      tail_packet->bytes[0] = packet_index++;

      if(tail_packet->next_packet != NULL) {
        tail_packet = tail_packet->next_packet;
      }
      else {
        break;
      }
    }
  }
  
  if(frame->data_section != NULL) {
    // Link to data section
    tail_packet->next_packet = frame->data_section;
    tail_packet = tail_packet->next_packet;

    // Iterate data section, find tail and add indexes
    for(;;) {
      tail_packet->bytes[0] = packet_index++;

      if(tail_packet->next_packet != NULL) {
        tail_packet = tail_packet->next_packet;
      }
      else {
        break;
      }
    }
  }
  header_packet->bytes[0] = packet_index + 1;
  frame->packet_prt = frame->header_packet;
}

downlink_packet_t *downlink_frame_get_packet(downlink_frame_t *frame) {
  downlink_frame_t *packet = frame->packet_prt;
  if(packet != NULL) {
    frame->packet_prt = frame->packet_prt->bytes;
    return packet;
  }
  else {
    return NULL;
  }
}
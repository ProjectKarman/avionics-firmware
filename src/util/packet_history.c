/*
 * packet_history.c
 *
 * Created: 10/12/2015 5:48:16 PM
 *  Author: Nigil Lee
 */ 

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "packet_history.h"

static void protocol_raw_frame_destory(struct protocol_raw_frame *frame);

void packet_history_init(packet_history_t *history) {
  memset(history, 0, sizeof(packet_history_t));
}

void packet_history_add(packet_history_t *history, struct protocol_raw_frame *frame) {
  if(history->current_size == PACKET_HISTORY_DEPTH) {
    protocol_raw_frame_destory(history->history[PACKET_HISTORY_DEPTH - 1]);
  }

  uint8_t i;
  for(i = PACKET_HISTORY_DEPTH; i > 0; i--) {
    // Shift arrangement
    history->history[i] = history->history[i - 1];
  }
  history->history[0] = frame;
}

static void protocol_raw_frame_destory(struct protocol_raw_frame *frame) {
  vPortFree(frame->packets);
  vPortFree(frame);
}
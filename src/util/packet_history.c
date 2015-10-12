/*
 * packet_history.c
 *
 * Created: 10/12/2015 5:48:16 PM
 *  Author: Nigil Lee
 */ 

#include <stdint.h>
#include <string.h>
#include "packet_history.h"

void packet_history_init(packet_history_t *history) {
  memset(history, 0, sizeof(packet_history_t));
}

void packet_history_add(packet_history_t *history, struct protocol_raw_frame frame) {
  if(history->current_size == PACKET_HISTORY_DEPTH) {
    // We should free the last item

  }
}
/*
 * message_types.h
 *
 * Created: 10/12/2015 8:47:11 PM
 *  Author: Nigil Lee
 */ 


#ifndef MESSAGE_TYPES_H_
#define MESSAGE_TYPES_H_

#include <stddef.h>
#include <stdint.h>

typedef struct {
  char *text;
  size_t len;
} general_message_t;

general_message_t *general_message_create(void);
void general_message_destory(general_message_t *message);

#endif /* MESSAGE_TYPES_H_ */
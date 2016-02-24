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

#define MESSAGE_MAX_SIZE 20
#define GENERAL_MESSAGE_TEXT_MAX_SIZE (MESSAGE_MAX_SIZE - 1)

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} l3g4200d_raw_xyz_t;

typedef struct {
  char text[GENERAL_MESSAGE_TEXT_MAX_SIZE];
  uint8_t len;
} general_message_t;

typedef struct {
  l3g4200d_raw_xyz_t gyro_data;	// Am I allowed to use this data type in this fashion?
  int32_t rocket_tempurature;
  int32_t rocket_pressure;
  uint32_t time_stamp;
} sensors_message_t;

general_message_t *general_message_create(void);
void general_message_destory(general_message_t *message);

#endif /* MESSAGE_TYPES_H_ */
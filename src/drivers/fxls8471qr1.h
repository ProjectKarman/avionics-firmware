/*
 * fxls8471qr1.h
 *
 * Created: 10/08/2015 4:54:05 PM
 *  Author: Brendan
 */ 


#ifndef FXLS8471QR1_H_
#define FXLS8471QR1_H_

#include <stdint.h>
#include <stdbool.h>

/* Public Types */
enum fxls8471qr1_fifo_mode {
	FXLS8471QR1_FIFO_OFF = 0x0,
	FXLS8471QR1_FIFO_CIRCULAR_BUFFER = 0x1,
	FXLS8471QR1_FIFO_STOP_ACCEPTING = 0x2,
	FXLS8471QR1_FIFO_TRIGGER = 0x3,
};

enum fxls8471qr1_fs_range {
	FXLS8471QR1_FS_RANGE_2G = 0x0,
	FXLS8471QR1_FS_RANGE_4G = 0x1,
	FXLS8471QR1_FS_RANGE_8G = 0x2,
};

enum fxls8471qr1_data_rate {
	FXLS8471QR1_DR_800_HZ = 0x0,
	FXLS8471QR1_DR_400_HZ = 0x1,
	FXLS8471QR1_DR_200_HZ = 0x2,
	FXLS8471QR1_DR_100_HZ = 0x3,
	FXLS8471QR1_DR_50_HZ = 0x4,
	FXLS8471QR1_DR_12_5_HZ = 0x5,
	FXLS8471QR1_DR_6_25_HZ = 0x6,
	FXLS8471QR1_DR_1_5625_HZ = 0x7,
};

enum fxls8471qr1_interrupt {
	FXLS8471QR1_INTERRUPT_ASLP = 0x80,
	FXLS8471QR1_INTERRUPT_FIFO = 0x40,
	FXLS8471QR1_INTERRUPT_TRANS = 0x20,
	FXLS8471QR1_INTERRUPT_LNDPRT = 0x10,
	FXLS8471QR1_INTERRUPT_PULSE = 0x08,
	FXLS8471QR1_INTERRUPT_FFMT = 0x04,
	FXLS8471QR1_INTERRUPT_A_VECM = 0x02,
	FXLS8471QR1_INTERRUPT_DRDY = 0x01,
};

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} fxls8471qr1_raw_accel_t;

typedef uint8_t fxls8471qr1_bitfield_t;

typedef void (*fxls8471qr1_callback_t)(void);
typedef void (*fxls8471qr1_data_callback_t)(fxls8471qr1_raw_accel_t);

/* Public Functions */
void fxls8471qr1_init(void);
uint8_t fxls8471qr1_set_fifo_mode(enum fxls8471qr1_fifo_mode fifo_mode);
uint8_t fxls8471qr1_set_fifo_watermark(uint8_t watermark);
uint8_t fxls8471qr1_set_fs_range(enum fxls8471qr1_fs_range fs_range);
uint8_t fxls8471qr1_set_hp_filter(bool is_enabled);
uint8_t fxls8471qr1_set_data_rate(enum fxls8471qr1_data_rate data_rate);
uint8_t fxls8471qr1_set_interrupts(fxls8471qr1_bitfield_t bitfield);
uint8_t fxls8471qr1_set_interrupt_route(fxls8471qr1_bitfield_t bitfield);
uint8_t fxls8471qr1_get_data(fxls8471qr1_raw_accel_t *data);
uint8_t fxls8471qr1_get_data_async(fxls8471qr1_data_callback_t callback);
uint8_t fxls8471qr1_get_data_async_from_isr(fxls8471qr1_data_callback_t callback);
uint8_t fxls8471qr1_activate(void);
void fxls8471qr1_set_int1_handler(fxls8471qr1_callback_t callback);
void fxls8471qr1_set_int2_handler(fxls8471qr1_callback_t callback);

#endif /* FXLS8471QR1_H_ */
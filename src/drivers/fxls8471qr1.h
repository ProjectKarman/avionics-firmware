/*
 * fxls8471qr1.h
 *
 * Created: 10/08/2015 4:54:05 PM
 *  Author: Brendan
 */ 


#ifndef FXLS8471QR1_H_
#define FXLS8471QR1_H_

#include <stdint.h>
#include <string.h>

/* Public Types */
enum fxls8471qr1_fifo_mode {
	FXLS8471QR1_FIFO_OFF = 0x0,
	FXLS8471QR1_FIFO_CIRCULAR_BUFFER = 0x1,
	FXLS8471QR1_FIFO_STOP_ACCEPTING = 0x2,
	FXLS8471QR1_FIFO_TRIGGER = 0x3,
};

enum fxls8471qr1_fs_range {
	FXLS8471QR1_FS_RANGE_244,
	FXLS8471QR1_FS_RANGE_488,
	FXLS8471QR1_FS_RANGE_976,
};

enum fxls8471qr1_ctrl_reg1_dr {
	FXLS8471QR1_DR_800_HZ,
	FXLS8471QR1_DR_400_HZ,
	FXLS8471QR1_DR_200_HZ,
	FXLS8471QR1_DR_100_HZ,
	FXLS8471QR1_DR_50_HZ,
	FXLS8471QR1_DR_12_5_HZ,
	FXLS8471QR1_DR_6_25_HZ,
	FXLS8471QR1_DR_1_5625_HZ,
};

enum fxls8471qr1_ctrl_reg4_interrupt {
	FXLS8471QR1_INTERRUPT_ASLP,
	FXLS8471QR1_INTERRUPT_FIFO,
	FXLS8471QR1_INTERRUPT_TRANS,
	FXLS8471QR1_INTERRUPT_LNDPRT,
	FXLS8471QR1_INTERRUPT_PULSE,
	FXLS8471QR1_INTERRUPT_FFMT,
	FXLS8471QR1_INTERRUPT_A_VECM,
	FXLS8471QR1_INTERRUPT_DRDY,
};

enum fxls8471qr1_ctrl_reg5_interrupt_route {
	FXLS8471QR1_INTERRUPT_RT_ASLP,
	FXLS8471QR1_INTERRUPT_RT_FIFO,
	FXLS8471QR1_INTERRUPT_RT_TRANS,
	FXLS8471QR1_INTERRUPT_RT_LNDPRT,
	FXLS8471QR1_INTERRUPT_RT_PULSE,
	FXLS8471QR1_INTERRUPT_RT_FFMT,
	FXLS8471QR1_INTERRUPT_RT_A_VECM,
	FXLS8471QR1_INTERRUPT_RT_DRDY,
};

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} fxls8471qr1_raw_accel_t;

typedef void (*fxls8471qr1_callback_t)(void);
typedef void (*fxls8471qr1_data_callback_t)(fxls8471qr1_raw_accel_t);

/* Public Functions */
void fxls8471qr1_init(void);
uint8_t fxls8471qr1_setup_fifo_mode(enum fxls8471qr1_fifo_mode fifo_mode);
uint8_t fxls8471qr1_setup_fifo_watermark(uint8_t watermark);
/*
void fxls8471qr1_setup_xyz_fs_range(enum fxls8471qr1_fs_range fs_range);
void fxls8471qr1_setup_xyz_hp(bool hp_on);
void fxls8471qr1_setup_ctrl_reg1_dr(enum fxls8471qr1_ctrl_reg1_dr data_rate);
void fxls8471qr1_setup_ctrl_reg4_interrupt_enable(enum fxls8471qr1_ctrl_reg4_interrupt interrupt, bool enable);
void fxls8471qr1_setup_ctrl_reg5_interrupt_route_toggle(enum fxls8471qr1_ctrl_reg5_interrupt_route route, bool pin);
void fxls8471qr1_record_latest_xyz_data(XYZ_DATA *xyzAccelData, int fifoReadNum);
*/
 
#endif /* FXLS8471QR1_H_ */
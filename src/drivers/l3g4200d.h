/*
 * l3g4200d.h
 *
 * Created: 11/8/2015 9:58:56 PM
 *  Author: Brendan
 */ 


#ifndef L3G4200D_H_
#define L3G4200D_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

enum l3g4200d_fifo_mode {
	L3G4200D_FIFO_BYPASS = 0x0,
	L3G4200D_FIFO_MODE = 0x1,
	L3G4200D_FIFO_STREAM = 0x2,
	L3G4200D_FIFO_STREAM_TO_FIFO = 0x3,
	L3G4200D_FIFO_BYPASS_TO_STREAM = 0x4,
};

enum l3g4200d_ctrl_reg1_odr_bw {
	L3G4200D_DR_100_BW_12_5 = 0x0,
	L3G4200D_DR_100_BW_25 = 0x1,
	L3G4200D_DR_200_BW_12_5 = 0x3,
	L3G4200D_DR_200_BW_25 = 0x4,
	L3G4200D_DR_200_BW_50 = 0x5,
	L3G4200D_DR_200_BW_70 = 0x6,
	L3G4200D_DR_400_BW_20 = 0x7,
	L3G4200D_DR_400_BW_25 = 0x8,
	L3G4200D_DR_400_BW_50 = 0x9,
	L3G4200D_DR_400_BW_110 = 0xA,
	L3G4200D_DR_800_BW_30 = 0xB,
	L3G4200D_DR_800_BW_35 = 0xC,
	L3G4200D_DR_800_BW_50 = 0xD,
	L3G4200D_DR_800_BW_110 = 0xE,
};

enum l3g4200d_ctrl_reg1_power_mode {
  L3G4200D_POWER_DOWN_MODE = 0x0,
  L3G4200D_SLEEP_NORMAL_MODE = 0x8
};

enum l3g4200d_ctrl_reg2_hp_mode {
	L3G4200D_HP_NORMAL_RESET = 0x0,
	L3G4200D_HP_REFERENCE_FILTER = 0x1,
	L3G4200D_HP_NORMAL = 0x2,
	L3G4200D_HP_AUTORESET = 0x3,
};
//The enum values remain the same, but hp cutoff values differ with different ODRs
enum l3g4200d_ctrl_reg2_hp_cutoff {
  L3G4200D_HP_CUTOFF_ODR_100HZ_8HZ = 0x0,
  L3G4200D_HP_CUTOFF_ODR_100HZ_4HZ = 0x1,
  L3G4200D_HP_CUTOFF_ODR_100HZ_2HZ = 0x2,
  L3G4200D_HP_CUTOFF_ODR_100HZ_1HZ = 0x3,
  L3G4200D_HP_CUTOFF_ODR_100HZ_0_5HZ = 0x4,
  L3G4200D_HP_CUTOFF_ODR_100HZ_0_2HZ = 0x5,
  L3G4200D_HP_CUTOFF_ODR_100HZ_0_1HZ = 0x6,
  L3G4200D_HP_CUTOFF_ODR_100HZ_0_05HZ = 0x7,
  L3G4200D_HP_CUTOFF_ODR_100HZ_0_02HZ = 0x8,
  L3G4200D_HP_CUTOFF_ODR_100HZ_0_01HZ = 0x9,
};

enum l3g4200d_ctrl_reg4_fs_select {
  L3G4200D_FS_SELECT_250_DPS = 0x0,
  L3G4200D_FS_SELECT_500_DPS = 0x1,
  L3G4200D_FS_SELECT_2000_DPS = 0x2
};

enum l3g4200d_ctrl_reg4_self_test_enable {
  L3G4200D_SELF_TEST_NORMAL_MODE = 0x0,
  L3G4200D_SELF_TEST_0_MODE = 0x1,
  L3G4200D_SELF_TEST_1_MODE = 0x3
};

enum l3g4200d_ctrl_reg4_spi_mode {
  L3G4200D_SPI_MODE_4_WIRE = 0x0,
  L3G4200D_SPI_MODE_3_WIRE = 0x1
};  

enum l3g4200d_ctrl_reg5_out_sel {
  L3G4200D_OUT_SEL_NON_HP_FILTERED = 0x0,
  L3G4200D_OUT_SEL_HP_FILTERED = 0x1,
  L3G4200D_OUT_SEL_LP_FILTERED = 0x2,
};

enum l3g4200d_ctrl_reg5_int1_sel {
  L3G4200D_INT1_SEL_NON_HP_FILTERED = 0x0,
  L3G4200D_INT1_SEL_HP_FILTERED = 0x1,
  L3G4200D_INT1_SEL_LP_FILTERED = 0x2
};

enum l3g4200d_ctrl_reg3_setup {
  L3G4200D_CTRL_REG3_INT1_EN = 0x80,
  L3G4200D_CTRL_REG3_INT1_BOOT_STATUS = 0x40,
  L3G4200D_CTRL_REG3_INT1_ACTIVE_H0_L1 = 0x20,
  L3G4200D_CTRL_REG3_PUSHPULL0_OPENDRAIN1 = 0x10,
  L3G4200D_CTRL_REG3_INT2_DATA_READY = 0x08,
  L3G4200D_CTRL_REG3_INT2_FIFO_WMRK_INTERRUPT = 0x04,
  L3G4200D_CTRL_REG3_INT2_FIFO_OVERRUN_INTERRUPT = 0x02,
  L3G4200D_CTRL_REG3_INT2_FIFO_EMPTY_INTERRUPT = 0x01,
};


typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} l3g4200d_raw_xyz_t;

typedef uint8_t l3g4200d_bitfield_t;
typedef void (*l3g4200d_callback_t)(void);
typedef void (*l3g4200d_data_callback_t)(l3g4200d_raw_xyz_t);

// Public Functions
void l3g4200d_init(void);

uint8_t l3g4200d_check_comms(void);
uint8_t l3g4200d_setup_fifo_mode(enum l3g4200d_fifo_mode fifo_mode);
uint8_t l3g4200d_setup_fifo_watermark(uint8_t watermark);
uint8_t l3g4200d_setup_ctrl_reg1_dr_bw(enum l3g4200d_ctrl_reg1_odr_bw odr_bw_select);
uint8_t l3g4200d_setup_ctrl_reg2_hp(enum l3g4200d_ctrl_reg2_hp_mode hp_mode);
uint8_t l3g4200d_setup_ctrl_reg2_hp_cutoff(enum l3g4200d_ctrl_reg2_hp_cutoff hp_cutoff);
uint8_t l3g4200d_setup_ctrl_reg3(l3g4200d_bitfield_t bitfield);
uint8_t l3g4200d_setup_ctrl_reg4_fs_select(enum l3g4200d_ctrl_reg4_fs_select fs_select);
uint8_t l3g4200d_setup_ctrl_reg4_self_test_enable(enum l3g4200d_ctrl_reg4_self_test_enable self_test);
uint8_t l3g4200d_setup_ctrl_reg4_spi_mode_select(enum l3g4200d_ctrl_reg4_spi_mode spi_mode);
uint8_t l3g4200d_setup_ctrl_reg5_fifo_enable(bool enable);
uint8_t l3g4200d_setup_ctrl_reg5_hp_filter_enable(bool enable);
uint8_t l3g4200d_setup_ctrl_reg5_int1_sel(enum l3g4200d_ctrl_reg5_int1_sel int1_sel);
uint8_t l3g4200d_setup_ctrl_reg5_out_sel(enum l3g4200d_ctrl_reg5_out_sel out_sel);
uint8_t l3g4200d_setup_int1_cfg(l3g4200d_bitfield_t bitfield);
uint8_t l3g4200d_setup_int1_ths_x(uint16_t threshold);
uint8_t l3g4200d_setup_int1_ths_y(uint16_t threshold);
uint8_t l3g4200d_setup_int1_ths_z(uint16_t threshold);
uint8_t l3g4200d_setup_int1_wait_enable(bool enable);
uint8_t l3g4200d_setup_int1_duration(l3g4200d_bitfield_t bitfield);
uint8_t l3g4200d_get_data(l3g4200d_raw_xyz_t *data);
uint8_t l3g4200d_get_data_async(l3g4200d_data_callback_t callback);
uint8_t l3g4200d_get_data_async_from_isr(l3g4200d_data_callback_t callback);
uint8_t l3g4200d_activate(void);
void l3g4200d_set_int1_handler(l3g4200d_callback_t callback);
void l3g4200d_set_int2_handler(l3g4200d_callback_t callback);


#endif /* L3G4200D_H_ */
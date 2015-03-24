/**
 * @file    l3gd20h.h
 * @brief   L3GD20H MEMS interface module through SPI header.
 *
 * @addtogroup l3gd20h
 * @{
 */

#ifndef _L3GD20H_H_
#define _L3GD20H_H_

#include "ch.h"
#include "hal.h"

/**
 * @name    L3GD20H register names
 * @{
 */
#define L3GD20H_WHO_AM_I			0x0F
#define L3GD20H_CTRL1				0x20
#define L3GD20H_CTRL2				0x21
#define L3GD20H_CTRL3				0x22
#define L3GD20H_CTRL4				0x23
#define L3GD20H_CTRL5				0x24
#define L3GD20H_REFERENCE			0x25
#define L3GD20H_OUT_TEMP			0x26
#define L3GD20H_STATUS				0x27
#define L3GD20H_OUT_X_L				0x28
#define L3GD20H_OUT_X_H				0x29
#define L3GD20H_OUT_Y_L				0x2A
#define L3GD20H_OUT_Y_H				0x2B
#define L3GD20H_OUT_Z_L				0x2C
#define L3GD20H_OUT_Z_H				0x2D
#define L3GD20H_FIFO_CTRL			0x2E
#define L3GD20H_FIFO_SRC			0x2F
#define L3GD20H_IG_CFG				0x30
#define L3GD20H_IG_SRC				0x31
#define L3GD20H_IG_TSH_XH			0x32
#define L3GD20H_IG_TSH_XL			0x33
#define L3GD20H_IG_TSH_YH			0x34
#define L3GD20H_IG_TSH_YL			0x35
#define L3GD20H_IG_TSH_ZH			0x36
#define L3GD20H_IG_TSH_ZL			0x37
#define L3GD20H_IG_DURATION			0x38
#define L3GD20H_LOW_ODR				0x39

#define L3GD20H_AXIS_X					0
#define L3GD20H_AXIS_Y					1
#define L3GD20H_AXIS_Z					2

#define GYRO_DATA_READY                 123

/** @} */

typedef struct {
  uint32_t t;
  int16_t x;
  int16_t y;
  int16_t z;
} gyro_data_t;

#ifdef __cplusplus
extern "C" {
#endif
  int init_l3gd20h(SPIDriver *spip);
  uint8_t l3gd20hReadRegister(SPIDriver *spip, uint8_t reg);
  void l3gd20hWriteRegister(SPIDriver *spip, uint8_t reg, uint8_t value);
  int16_t l3gd20hGetAxis(SPIDriver *spip, uint8_t axis);
  void l3gd20h_update(SPIDriver *spip);
  void l3gd20h_drdy_callback(EXTDriver *extp, expchannel_t channel);
  Thread *gyroRun(SPIDriver *spip, tprio_t prio);
#ifdef __cplusplus
}
#endif

#endif /* _L3GD20H_H_ */

/** @} */

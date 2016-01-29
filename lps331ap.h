/**
 * @file    lps331ap.h
 * @brief   LPS331AP MEMS interface module through SPI header.
 *
 * @addtogroup lps331ap
 * @{
 */

#ifndef _LPS331AP_H_
#define _LPS331AP_H_

#include "ch.h"
#include "hal.h"

/**
 * @name    LPS331AP register names
 * @{
 */
#define LSP331AP_REF_P_XL			0x08
#define LSP331AP_REF_P_L			0x09
#define LSP331AP_REF_P_H			0x0A
#define LSP331AP_WHO_AM_I			0x0F
#define LPS331AP_RES_CONF			0x10
#define LPS331AP_CTRL_REG1			0x20
#define LPS331AP_CTRL_REG2			0x21
#define LPS331AP_CTRL_REG3			0x22
#define LSP331AP_INT_CFG_REG		0x23
#define LSP331AP_INT_SOURCE_REG		0x24
#define LSP331AP_THS_P_LOW_REG		0x25
#define LSP331AP_THS_P_HIGH_REG		0x26
#define LSP331AP_STATUS_REG			0x27
#define LSP331AP_PRESS_POUT_XL_REG	0x28
#define LSP331AP_PRESS_OUT_L		0x29
#define LSP331AP_PRESS_OUT_H		0x2A
#define LSP331AP_TEMP_OUT_L			0x2B
#define LSP331AP_TEMP_OUT_H			0x2C
#define LSP331AP_AMP_CTRL			0x30

/** @} */

typedef struct {
  uint32_t t;
  int16_t pressure;
} baro_data_t;

#ifdef __cplusplus
extern "C" {
#endif
  int init_lps331ap(SPIDriver *spip);
  uint8_t lps331apReadRegister(SPIDriver *spip, uint8_t reg);
  void lps331apWriteRegister(SPIDriver *spip, uint8_t reg, uint8_t value);
  void lps331ap_update(SPIDriver *spip);
  void lps331ap_drdy_callback(EXTDriver *extp, expchannel_t channel);
  Thread *baroRun(SPIDriver *spip, tprio_t prio);
#ifdef __cplusplus
}
#endif

#endif /* _LPS331AP_H_ */

/** @} */

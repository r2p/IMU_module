/**
 * @file    lps331ap.c
 * @brief   LPS331AP MEMS interface module through SPI code.
 *
 * @addtogroup lps331ap
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "lps331ap.h"

#define BAR_WA_SIZE    THD_WA_SIZE(256)

static uint8_t txbuf[2];
static uint8_t rxbuf[2];

static Thread *bartp = NULL;

bar_data_t bar_data = 0;

/**
 * @brief   Reads a register value.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI initerface
 * @param[in] reg       register number
 * @return              The register value.
 */
int init_lps331ap(SPIDriver *spip) {
	uint8_t who_am_i;

	spiAcquireBus(spip);
	who_am_i = lps331apReadRegister(spip, LSP331AP_WHO_AM_I);

	if (who_am_i != 0xBB) {
		spiReleaseBus(spip);
		return -1;
	}

//	lps331apWriteRegister(spip, L3GD20H_CTRL1, 0x7F);
//	lps331apWriteRegister(spip, L3GD20H_CTRL2, 0x00);
//	lps331apWriteRegister(spip, L3GD20H_CTRL3, 0x08);
//	lps331apWriteRegister(spip, L3GD20H_CTRL4, 0x10);
//	lps331apWriteRegister(spip, L3GD20H_CTRL5, 0x00);
	spiReleaseBus(spip);

	chThdSleepMilliseconds(250);
	return 0;
}

/**
 * @brief   Reads a register value.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI initerface
 * @param[in] reg       register number
 * @return              The register value.
 */
uint8_t lps331apReadRegister(SPIDriver *spip, uint8_t reg) {

	palClearPad(GPIOB, GPIOB_BAR_CS);
	txbuf[0] = 0x80 | reg;
	txbuf[1] = 0xff;
	spiExchange(spip, 2, txbuf, rxbuf);
	palSetPad(GPIOB, GPIOB_BAR_CS);
	return rxbuf[1];
}

/**
 * @brief   Writes a value into a register.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI initerface
 * @param[in] reg       register number
 * @param[in] value     the value to be written
 */
void lps331apWriteRegister(SPIDriver *spip, uint8_t reg, uint8_t value) {

	switch (reg) {
	case LSP331AP_WHO_AM_I:
		/* Read only registers cannot be written, the command is ignored.*/
		return;
	case LPS331AP_CTRL_REG1:
	case LPS331AP_CTRL_REG2:
	case LPS331AP_CTRL_REG3:
		palClearPad(GPIOB, GPIOB_BAR_CS);
		txbuf[0] = reg;
		txbuf[1] = value;
		spiSend(spip, 2, txbuf);
		palSetPad(GPIOB, GPIOB_BAR_CS);
		break;
	default:
		/* Reserved register must not be written, according to the datasheet
		 this could permanently damage the device.*/
		chDbgAssert(FALSE, "lps331apWriteRegister(), #1", "reserved register");
		break;
	}
}


//void lps331ap_update(SPIDriver *spip) {
//	int16_t data[3];
//	systime_t timestamp;
//
//	timestamp = chTimeNow();
//
//	//XXX da fare lettura sequenziale!
//	spiAcquireBus(spip);
//	data[0] = (lps331apReadRegister(spip, L3GD20H_OUT_X_H) & 0xFF) << 8;
//	data[0] |= lps331apReadRegister(spip, L3GD20H_OUT_X_L) & 0xFF;
//	data[1] = (lps331apReadRegister(spip, L3GD20H_OUT_Y_H) & 0xFF) << 8;
//	data[1] |= lps331apReadRegister(spip, L3GD20H_OUT_Y_L) & 0xFF;
//	data[2] = (lps331apReadRegister(spip, L3GD20H_OUT_Z_H) & 0xFF) << 8;
//	data[2] |= lps331apReadRegister(spip, L3GD20H_OUT_Z_L) & 0xFF;
//	spiReleaseBus(spip);
//
//	chSysLock();
//	gyro_data.t = timestamp;
//	gyro_data.x = data[0];
//	gyro_data.y = data[1];
//	gyro_data.z = data[2];
//	chSysUnlock();
//}
//
//void lps331ap_drdy_callback(EXTDriver *extp, expchannel_t channel) {
//	(void) extp;
//	(void) channel;
//
//	/* Wakes up the thread.*/chSysLockFromIsr()
//	;
//	if (gyrotp != NULL) {
//		gyrotp->p_u.rdymsg = (msg_t) BARO_DATA_READY;
//		chSchReadyI(gyrotp);
//		gyrotp = NULL;
//	}
//	chSysUnlockFromIsr();
//}
//
//static msg_t lps331ap_update_thread(void *p) {
//	SPIDriver *spip = (SPIDriver *) p;
//
//	while (TRUE) {
//		msg_t msg;
//
//		/* Waiting for the IRQ to happen.*/chSysLock()
//		;
//		gyrotp = chThdSelf();
//		chSchGoSleepS(THD_STATE_SUSPENDED);
//		msg = chThdSelf()->p_u.rdymsg;
//		chSysUnlock();
//
//		/* If data ready, update all axis.*/
//		if (msg == BARO_DATA_READY) {
//			lps331ap_update(spip);
//		}
//	}
//
//	return RDY_OK;
//}
//
//Thread *gyroRun(SPIDriver *spip, tprio_t prio) {
//	Thread *tp;
//
//	if (init_lps331ap(spip) != 0) chSysHalt();
//
//	tp = chThdCreateFromHeap(NULL, GYRO_WA_SIZE, prio, lps331ap_update_thread, (void*) spip);
//	extChannelEnable(&EXTD1, GPIOB_GYRO_INT2);
//	lps331ap_update(spip);
//
//	return tp;
//}

/** @} */

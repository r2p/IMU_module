#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "lsm303d.h"

#define ACC_WA_SIZE    THD_WA_SIZE(256)
#define MAG_WA_SIZE    THD_WA_SIZE(256)

static uint8_t txbuf[2];
static uint8_t rxbuf[2];

static Thread *acctp = NULL;
static Thread *magtp = NULL;

acc_data_t acc_data = { 0, 0, 0, 0 };
acc_data_t mag_data = { 0, 0, 0, 0 };

/**
 * @brief   Reads a register value.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI initerface
 * @param[in] reg       register number
 * @return              The register value.
 */
uint8_t lsm303dReadRegister(SPIDriver *spip, uint8_t reg) {

	palClearPad(GPIOA, GPIOA_AM_CS);
	txbuf[0] = 0x80 | reg;
	txbuf[1] = 0xff;
	spiExchange(spip, 2, txbuf, rxbuf);
	palSetPad(GPIOA, GPIOA_AM_CS);
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
void lsm303dWriteRegister(SPIDriver *spip, uint8_t reg, uint8_t value) {

	switch (reg) {
	case LSM303D_TEMP_OUT_L:
	case LSM303D_TEMP_OUT_H:
	case LSM303D_STATUS_M:
	case LSM303D_OUT_X_L_M:
	case LSM303D_OUT_X_H_M:
	case LSM303D_OUT_Y_L_M:
	case LSM303D_OUT_Y_H_M:
	case LSM303D_OUT_Z_L_M:
	case LSM303D_OUT_Z_H_M:
	case LSM303D_WHO_AM_I:
	case LSM303D_INT_SRC_M:
	case LSM303D_STATUS_A:
	case LSM303D_OUT_X_L_A:
	case LSM303D_OUT_X_H_A:
	case LSM303D_OUT_Y_L_A:
	case LSM303D_OUT_Y_H_A:
	case LSM303D_OUT_Z_L_A:
	case LSM303D_OUT_Z_H_A:
	case LSM303D_FIFO_SRC:
	case LSM303D_IG_SRC1:
	case LSM303D_IG_SRC2:
	case LSM303D_CLICK_SRC:
		/* Read only registers cannot be written, the command is ignored.*/
		return;
	case LSM303D_CTRL_M:
	case LSM303D_THS_L_M:
	case LSM303D_THS_H_M:
	case LSM303D_OFFSET_X_L_M:
	case LSM303D_OFFSET_X_H_M:
	case LSM303D_OFFSET_Y_L_M:
	case LSM303D_OFFSET_Y_H_M:
	case LSM303D_OFFSET_Z_L_M:
	case LSM303D_OFFSET_Z_H_M:
	case LSM303D_REFERENCE_X:
	case LSM303D_REFERENCE_Y:
	case LSM303D_REFERENCE_Z:
	case LSM303D_CTRL0:
	case LSM303D_CTRL1:
	case LSM303D_CTRL2:
	case LSM303D_CTRL3:
	case LSM303D_CTRL4:
	case LSM303D_CTRL5:
	case LSM303D_CTRL6:
	case LSM303D_CTRL7:
	case LSM303D_FIFO_CTRL:
	case LSM303D_IG_CFG1:
	case LSM303D_IG_THS1:
	case LSM303D_IG_DUR1:
	case LSM303D_IG_CFG2:
	case LSM303D_IG_THS2:
	case LSM303D_IG_DUR2:
	case LSM303D_CLICK_CFG:
	case LSM303D_CLICK_THS:
	case LSM303D_TIME_LIMIT:
	case LSM303D_TIME_LATENCY:
	case LSM303D_TIME_WINDOW:
	case LSM303D_ACT_THS:
	case LSM303D_ACT_DUR:
		palClearPad(GPIOA, GPIOA_AM_CS);
		txbuf[0] = reg;
		txbuf[1] = value;
		spiSend(spip, 2, txbuf);
		palSetPad(GPIOA, GPIOA_AM_CS);
		break;
	default:
		/* Reserved register must not be written, according to the datasheet
		 this could permanently damage the device.*/
		chDbgAssert(FALSE, "lsm303dWriteRegister(), #1", "reserved register");
		break;
	}
}

/**
 * Init function.
 */
int lsm303_init(SPIDriver *spip) {
	uint8_t who_am_i;

	// FIXME: interrupt lines will be rearranged!
	spiAcquireBus(spip);
	who_am_i = lsm303dReadRegister(spip, LSM303D_WHO_AM_I);

	if (who_am_i != 0x49) {
		spiReleaseBus(spip);
		return -1;
	}

	lsm303dWriteRegister(spip, LSM303D_CTRL0, 0x00);
	lsm303dWriteRegister(spip, LSM303D_CTRL2, 0x40); // 194Hz anti-alias filter BW
	lsm303dWriteRegister(spip, LSM303D_CTRL3, 0x04); // Acc data-ready on INT1
	lsm303dWriteRegister(spip, LSM303D_CTRL4, 0x04); // Mag data-ready on INT2
	lsm303dWriteRegister(spip, LSM303D_CTRL5, 0xF4); // Temp sensor enabled, magnetometer high-res, Mag 100Hz
	lsm303dWriteRegister(spip, LSM303D_CTRL6, 0x00); // Mag FS ±2 gauss
	lsm303dWriteRegister(spip, LSM303D_CTRL7, 0x00);
	lsm303dWriteRegister(spip, LSM303D_CTRL1, 0x67); // Acc 100Hz, axes enabled, no data update until MSB and LSB read
	spiReleaseBus(spip);

	chThdSleepMilliseconds(250);
	return 0;
}

void lsm303_acc_update(SPIDriver *spip) {
	uint8_t txbuf;
	uint8_t rxbuf[6];
	systime_t timestamp;

	timestamp = chTimeNow();

	txbuf = 0x80 | 0x40 | LSM303D_OUT_X_L_A;
	spiAcquireBus(spip);
	palClearPad(GPIOA, GPIOA_AM_CS);
	spiSend(spip, 1, &txbuf);
	spiReceive(spip, 6, &rxbuf);
	palSetPad(GPIOA, GPIOA_AM_CS);
	spiReleaseBus(spip);

	chSysLock();
	acc_data.t = timestamp;
	acc_data.x = *((int16_t*)&(rxbuf[0])) >> 4;
    acc_data.y = *((int16_t*)&(rxbuf[2])) >> 4;
	acc_data.z = *((int16_t*)&(rxbuf[4])) >> 4;
	chSysUnlock();
}

void lsm303_int1_cb(EXTDriver *extp, expchannel_t channel) {
	(void) extp;
	(void) channel;

	/* Wakes up the thread.*/chSysLockFromIsr()
	;
	if (acctp != NULL) {
		acctp->p_u.rdymsg = (msg_t) LSM303D_ACC_DATA_READY;
		chSchReadyI(acctp);
		acctp = NULL;
	}
	chSysUnlockFromIsr();
}

static msg_t lsm303_acc_update_thread(void *p) {
	SPIDriver *spip = (SPIDriver *) p;

	while (TRUE) {
		msg_t msg;

		/* Waiting for the IRQ to happen.*/

		chSysLock();
		acctp = chThdSelf();
		chSchGoSleepS(THD_STATE_SUSPENDED);
		msg = chThdSelf()->p_u.rdymsg;
		chSysUnlock();

		/* If data ready, update all axis.*/
		if (msg == LSM303D_ACC_DATA_READY) {
			lsm303_acc_update(spip);
		}

		lsm303_acc_update(spip);
	}

	return RDY_OK;
}

/**
 *
 */
void lsm303_mag_update(SPIDriver *spip) {
	uint8_t txbuf;
	uint8_t rxbuf[6];
	systime_t timestamp;

	timestamp = chTimeNow();

	txbuf = 0x80 | 0x40 | LSM303D_OUT_X_L_M;
	spiAcquireBus(spip);
	palClearPad(GPIOA, GPIOA_AM_CS);
	spiSend(spip, 1, &txbuf);
	spiReceive(spip, 6, &rxbuf);
	palSetPad(GPIOA, GPIOA_AM_CS);
	spiReleaseBus(spip);

	chSysLock();
	mag_data.t = timestamp;
	mag_data.x = *((int16_t*)&(rxbuf[0])) >> 4;
    mag_data.y = *((int16_t*)&(rxbuf[2])) >> 4;
	mag_data.z = *((int16_t*)&(rxbuf[4])) >> 4;
	chSysUnlock();
}

void lsm303_int2_cb(EXTDriver *extp, expchannel_t channel) {
	(void) extp;
	(void) channel;

	/* Wakes up the thread.*/chSysLockFromIsr()
	;
	if (magtp != NULL) {
		magtp->p_u.rdymsg = (msg_t) LSM303D_MAG_DATA_READY;
		chSchReadyI(magtp);
		magtp = NULL;
	}
	chSysUnlockFromIsr();
}

static msg_t lsm303_mag_update_thread(void *p) {
	SPIDriver *spip = (SPIDriver *) p;

	while (TRUE) {
		msg_t msg;

		/* Waiting for the IRQ to happen.*/

		chSysLock();
		magtp = chThdSelf();
		chSchGoSleepS(THD_STATE_SUSPENDED);
		msg = chThdSelf()->p_u.rdymsg;
		chSysUnlock();

		/* If data ready, update all axis.*/
		if (msg == LSM303D_MAG_DATA_READY) {
			lsm303_mag_update(spip);
		}
	}

	return RDY_OK;
}

/*
 * Threads.
 */

Thread *accRun(SPIDriver *spip, tprio_t prio) {
	Thread *tp;

	if (lsm303_init(spip) != 0) chSysHalt();

	chThdSleepMilliseconds(200);
	lsm303_acc_update(spip);
	tp = chThdCreateFromHeap(NULL, ACC_WA_SIZE, prio, lsm303_acc_update_thread, (void*) spip);
	extChannelEnable(&EXTD1, GPIOA_AM_INT1);

	return tp;
}

Thread *magRun(SPIDriver *spip, tprio_t prio) {
	Thread *tp;

//	lsm303_mag_init(spip);
	chThdSleepMilliseconds(200);
	lsm303_mag_update(spip);
	tp = chThdCreateFromHeap(NULL, MAG_WA_SIZE, prio, lsm303_mag_update_thread, (void*) spip);
	extChannelEnable(&EXTD1, GPIOA_AM_INT2);

	return tp;
}


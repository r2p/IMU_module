/**
 * @file    l3gd20h.c
 * @brief   L3GD20H MEMS interface module through SPI code.
 *
 * @addtogroup l3gd20h
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "l3gd20h.h"

#define GYRO_WA_SIZE    THD_WA_SIZE(256)

static uint8_t txbuf[2];
static uint8_t rxbuf[2];

static Thread *gyrotp = NULL;

gyro_data_t gyro_data = { 0, 0, 0, 0 };

/**
 * @brief   Reads a register value.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI initerface
 * @param[in] reg       register number
 * @return              The register value.
 */
int init_l3gd20h(SPIDriver *spip) {
	uint8_t who_am_i;

	spiAcquireBus(spip);
	who_am_i = l3gd20hReadRegister(spip, L3GD20H_WHO_AM_I);

	if (who_am_i != 0xD7) {
		spiReleaseBus(spip);
		return -1;
	}

	l3gd20hWriteRegister(spip, L3GD20H_CTRL1, 0x7F);
	l3gd20hWriteRegister(spip, L3GD20H_CTRL2, 0x00);
	l3gd20hWriteRegister(spip, L3GD20H_CTRL3, 0x08);
	l3gd20hWriteRegister(spip, L3GD20H_CTRL4, 0x10);
	l3gd20hWriteRegister(spip, L3GD20H_CTRL5, 0x00);
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
uint8_t l3gd20hReadRegister(SPIDriver *spip, uint8_t reg) {

	palClearPad(GPIOB, GPIOB_GYRO_CS);
	txbuf[0] = 0x80 | reg;
	txbuf[1] = 0xff;
	spiExchange(spip, 2, txbuf, rxbuf);
	palSetPad(GPIOB, GPIOB_GYRO_CS);
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
void l3gd20hWriteRegister(SPIDriver *spip, uint8_t reg, uint8_t value) {

	switch (reg) {
	case L3GD20H_WHO_AM_I:
	case L3GD20H_OUT_TEMP:
	case L3GD20H_STATUS:
	case L3GD20H_OUT_X_L:
	case L3GD20H_OUT_X_H:
	case L3GD20H_OUT_Y_L:
	case L3GD20H_OUT_Y_H:
	case L3GD20H_OUT_Z_L:
	case L3GD20H_OUT_Z_H:
		/* Read only registers cannot be written, the command is ignored.*/
		return;
	case L3GD20H_CTRL1:
	case L3GD20H_CTRL2:
	case L3GD20H_CTRL3:
	case L3GD20H_CTRL4:
	case L3GD20H_CTRL5:
	case L3GD20H_REFERENCE:
	case L3GD20H_FIFO_CTRL:
	case L3GD20H_FIFO_SRC:
	case L3GD20H_IG_CFG:
	case L3GD20H_IG_SRC:
	case L3GD20H_IG_TSH_XH:
	case L3GD20H_IG_TSH_XL:
	case L3GD20H_IG_TSH_YH:
	case L3GD20H_IG_TSH_YL:
	case L3GD20H_IG_TSH_ZH:
	case L3GD20H_IG_TSH_ZL:
	case L3GD20H_IG_DURATION:
	case L3GD20H_LOW_ODR:
		palClearPad(GPIOB, GPIOB_GYRO_CS);
		txbuf[0] = reg;
		txbuf[1] = value;
		spiSend(spip, 2, txbuf);
		palSetPad(GPIOB, GPIOB_GYRO_CS);
		break;
	default:
		/* Reserved register must not be written, according to the datasheet
		 this could permanently damage the device.*/
		chDbgAssert(FALSE, "L3GD20HWriteRegister(), #1", "reserved register");
		break;
	}
}

int16_t l3gd20hGetAxis(SPIDriver *spip, uint8_t axis) {
	int16_t data = 0;

	switch (axis) {
	case L3GD20H_AXIS_X:
		data = (l3gd20hReadRegister(spip, L3GD20H_OUT_X_H) & 0xFF) << 8;
		data |= l3gd20hReadRegister(spip, L3GD20H_OUT_X_L) & 0xFF;
		break;
	case L3GD20H_AXIS_Y:
		data = (l3gd20hReadRegister(spip, L3GD20H_OUT_Y_H) & 0xFF) << 8;
		data |= l3gd20hReadRegister(spip, L3GD20H_OUT_Y_L) & 0xFF;
		break;
	case L3GD20H_AXIS_Z:
		data = (l3gd20hReadRegister(spip, L3GD20H_OUT_Z_H) & 0xFF) << 8;
		data |= l3gd20hReadRegister(spip, L3GD20H_OUT_Z_L) & 0xFF;
		break;
	}

	return data;
}

void l3gd20h_update(SPIDriver *spip) {
	int16_t data[3];
	systime_t timestamp;

	timestamp = chTimeNow();

	//XXX da fare lettura sequenziale!
	spiAcquireBus(spip);
	data[0] = (l3gd20hReadRegister(spip, L3GD20H_OUT_X_H) & 0xFF) << 8;
	data[0] |= l3gd20hReadRegister(spip, L3GD20H_OUT_X_L) & 0xFF;
	data[1] = (l3gd20hReadRegister(spip, L3GD20H_OUT_Y_H) & 0xFF) << 8;
	data[1] |= l3gd20hReadRegister(spip, L3GD20H_OUT_Y_L) & 0xFF;
	data[2] = (l3gd20hReadRegister(spip, L3GD20H_OUT_Z_H) & 0xFF) << 8;
	data[2] |= l3gd20hReadRegister(spip, L3GD20H_OUT_Z_L) & 0xFF;
	spiReleaseBus(spip);

	chSysLock();
	gyro_data.t = timestamp;
	gyro_data.x = data[0];
	gyro_data.y = data[1];
	gyro_data.z = data[2];
	chSysUnlock();
}

void l3gd20h_drdy_callback(EXTDriver *extp, expchannel_t channel) {
	(void) extp;
	(void) channel;

	/* Wakes up the thread.*/chSysLockFromIsr()
	;
	if (gyrotp != NULL) {
		gyrotp->p_u.rdymsg = (msg_t) GYRO_DATA_READY;
		chSchReadyI(gyrotp);
		gyrotp = NULL;
	}
	chSysUnlockFromIsr();
}

static msg_t l3gd20h_update_thread(void *p) {
	SPIDriver *spip = (SPIDriver *) p;

	while (TRUE) {
		msg_t msg;

		/* Waiting for the IRQ to happen.*/chSysLock()
		;
		gyrotp = chThdSelf();
		chSchGoSleepS(THD_STATE_SUSPENDED);
		msg = chThdSelf()->p_u.rdymsg;
		chSysUnlock();

		/* If data ready, update all axis.*/
		if (msg == GYRO_DATA_READY) {
			l3gd20h_update(spip);
		}
	}

	return RDY_OK;
}

Thread *gyroRun(SPIDriver *spip, tprio_t prio) {
	Thread *tp;

	if (init_l3gd20h(spip) != 0) chSysHalt();

	tp = chThdCreateFromHeap(NULL, GYRO_WA_SIZE, prio, l3gd20h_update_thread, (void*) spip);
	extChannelEnable(&EXTD1, GPIOB_GYRO_INT2);
	l3gd20h_update(spip);

	return tp;
}

/** @} */

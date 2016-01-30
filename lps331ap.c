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

#define BARO_WA_SIZE    THD_WA_SIZE(256)
#define BARO_DATA_READY                 789

static uint8_t txbuf[2];
static uint8_t rxbuf[2];

static Thread *barotp = NULL;

baro_data_t baro_data = { 0, 0, 0};

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

	lps331apWriteRegister(spip, LPS331AP_CTRL_REG1, 0x00);
	lps331apWriteRegister(spip, LPS331AP_RES_CONF, 0x6A);
	lps331apWriteRegister(spip, LPS331AP_CTRL_REG2, 0x00);
	lps331apWriteRegister(spip, LPS331AP_CTRL_REG3, 0x04);
	lps331apWriteRegister(spip, LPS331AP_CTRL_REG1, 0xF8);
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

	palClearPad(GPIOB, GPIOB_BARO_CS);
	txbuf[0] = 0x80 | reg;
	txbuf[1] = 0xff;
	spiExchange(spip, 2, txbuf, rxbuf);
	palSetPad(GPIOB, GPIOB_BARO_CS);
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
	case LSP331AP_REF_P_XL:
	case LSP331AP_REF_P_L:
	case LSP331AP_REF_P_H:
	case LPS331AP_RES_CONF:
	case LPS331AP_CTRL_REG1:
	case LPS331AP_CTRL_REG2:
	case LPS331AP_CTRL_REG3:
	case LSP331AP_INT_CFG_REG:
	case LSP331AP_THS_P_LOW_REG:
	case LSP331AP_THS_P_HIGH_REG:
	case LSP331AP_AMP_CTRL:
		palClearPad(GPIOB, GPIOB_BARO_CS);
		txbuf[0] = reg;
		txbuf[1] = value;
		spiSend(spip, 2, txbuf);
		palSetPad(GPIOB, GPIOB_BARO_CS);
		break;
	default:
		/* Reserved register must not be written, according to the datasheet
		 this could permanently damage the device.*/
		chDbgAssert(FALSE, "lps331apWriteRegister(), #1", "reserved register");
		break;
	}
}


void lps331ap_update(SPIDriver *spip) {
	int32_t pressure;
	int16_t temperature;
	systime_t timestamp;

	timestamp = chTimeNow();

	//XXX da fare lettura sequenziale!
	spiAcquireBus(spip);
	pressure = lps331apReadRegister(spip, LSP331AP_PRESS_OUT_H) << 16;
	pressure |= lps331apReadRegister(spip, LSP331AP_PRESS_OUT_L) << 8;
	pressure |= lps331apReadRegister(spip, LSP331AP_PRESS_POUT_XL);
	temperature = lps331apReadRegister(spip, LSP331AP_TEMP_OUT_H) << 8;
	temperature |= lps331apReadRegister(spip, LSP331AP_TEMP_OUT_L);
	spiReleaseBus(spip);

	chSysLock();
	baro_data.t = timestamp;
	baro_data.pressure = pressure / 4096.0;
	baro_data.temperature = 42.5 + (temperature / 480.0);
	chSysUnlock();
}

void lps331ap_drdy_callback(EXTDriver *extp, expchannel_t channel) {
	(void) extp;
	(void) channel;

	/* Wakes up the thread.*/chSysLockFromIsr()
	;
	if (barotp != NULL) {
		barotp->p_u.rdymsg = (msg_t) BARO_DATA_READY;
		chSchReadyI(barotp);
		barotp = NULL;
	}
	chSysUnlockFromIsr();
}

static msg_t lps331ap_update_thread(void *p) {
	SPIDriver *spip = (SPIDriver *) p;

	while (TRUE) {
		msg_t msg;

#if 1
		/* Waiting for the IRQ to happen.*/chSysLock()
		;
		barotp = chThdSelf();
		chSchGoSleepS(THD_STATE_SUSPENDED);
		msg = chThdSelf()->p_u.rdymsg;
		chSysUnlock();

		/* If data ready, update all axis.*/
		if (msg == BARO_DATA_READY) {
			lps331ap_update(spip);
		}
# else
		chThdSleepMilliseconds(100);
		lps331ap_update(spip);
#endif
	}

	return RDY_OK;
}

Thread *baroRun(SPIDriver *spip, tprio_t prio) {
	Thread *tp;

	if (init_lps331ap(spip) != 0) chSysHalt();

	tp = chThdCreateFromHeap(NULL, BARO_WA_SIZE, prio, lps331ap_update_thread, (void*) spip);
	extChannelEnable(&EXTD1, GPIOA_BARO_INT1);
	lps331ap_update(spip);

	return tp;
}

/** @} */

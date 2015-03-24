#include <cstdio>
#include "ch.h"
#include "hal.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>
#include <r2p/msg/imu.hpp>
#include <r2p/msg/std_msgs.hpp>

#include "l3gd20h.h"
#include "lsm303d.h"
#include "lps331ap.h"
#include "madgwick.h"

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "IMU"
#endif

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

/*
 * Power supply voltage check.
 */
#define ADC_NUM_CHANNELS 1
#define ADC_BUF_DEPTH 2

static adcsample_t adc_buf[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 2 channels, SW triggered.
 * Channels:    IN7, IN8.
 */
static const ADCConversionGroup adcgrp_cfg = {
  FALSE,
  ADC_NUM_CHANNELS,
  NULL,
  NULL,
  0,                        /* CFGR    */
  ADC_TR(0, 4095),          /* TR1     */
  ADC_CCR_VBATEN,           /* CCR     */
  {                         /* SMPR[2] */
    0,
	ADC_SMPR2_SMP_AN17(ADC_SMPR_SMP_601P5)
  },
  {                         /* SQR[4]  */
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN17),
    0,
    0,
    0
  }
};


/*
 * Madgwick node
 */
extern gyro_data_t gyro_data;
extern acc_data_t acc_data;
extern mag_data_t mag_data;

static const SPIConfig spi1cfg = { 0x00, 0x00, 0x00, SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA, 0x00 };

static const EXTConfig extcfg = { {
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOB, l3gd20h_drdy_callback },
	{ EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, lsm303_int1_cb },
	{ EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, lsm303_int2_cb },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL }
} };
msg_t madgwick_node(void *arg) {
	r2p::Node node("madgwick");
	r2p::Publisher<r2p::IMUMsg> imu_pub;
	r2p::Publisher<r2p::IMURaw9> raw_pub;
	attitude_t attitude_data;
	systime_t time;

	(void) arg;
	chRegSetThreadName("madgwick");

	spiStart(&SPID1, &spi1cfg);
	extStart(&EXTD1, &extcfg);

	chThdSleepMilliseconds(500);

	gyroRun(&SPID1, NORMALPRIO);
	accRun(&SPID1, NORMALPRIO);
	magRun(&SPID1, NORMALPRIO);

	node.advertise(imu_pub, "imu");
	node.advertise(raw_pub, "imu_raw");

	time = chTimeNow();

	for (;;) {
		MadgwickAHRSupdate((gyro_data.y / 57.143) * 3.141592 / 180.0, (-gyro_data.x / 57.143) * 3.141592 / 180.0,
				(gyro_data.z / 57.143) * 3.141592 / 180.0, acc_data.x / 1000.0, acc_data.y / 1000.0,
				acc_data.z / 1000.0, mag_data.x, mag_data.y, mag_data.z);
		getMadAttitude(&attitude_data);

		r2p::IMUMsg *msgp;
		if (imu_pub.alloc(msgp)) {
			msgp->roll = ((attitude_data.roll * 57.29578)); // rads to degrees
			msgp->pitch = ((attitude_data.pitch * 57.29578)); // rads to degrees
			msgp->yaw = ((attitude_data.yaw * 57.29578)); // rads to degrees

			imu_pub.publish(*msgp);
		}

		r2p::IMURaw9 *rawp;
		if (raw_pub.alloc(rawp)) {
			rawp->acc_x = acc_data.x;
			rawp->acc_y = acc_data.y;
			rawp->acc_z = acc_data.z;
			rawp->gyro_x = gyro_data.y;
			rawp->gyro_y = -gyro_data.x;
			rawp->gyro_z = gyro_data.z;
			rawp->mag_x = mag_data.x;
			rawp->mag_y = mag_data.y;
			rawp->mag_z = mag_data.z;

			raw_pub.publish(*rawp);
		}

		time += MS2ST(20);
		chThdSleepUntil(time);
	}
	return CH_SUCCESS;
}

/*
 * Test node
 */

msg_t test_pub_node(void *arg) {
	r2p::Node node("test_pub");
	r2p::Publisher<r2p::String64Msg> pub;
	r2p::String64Msg * msgp;
	uint16_t * uuid = (uint16_t *)0x1FFFF7AC;
	float voltage;
	bool accmag_fail = false;
	bool gyro_fail= false;
	bool bar_fail= false;

	(void) arg;
	chRegSetThreadName("test_pub");

	node.advertise(pub, "test");
	chThdSleepMilliseconds(500);

	while (!pub.alloc(msgp)) chThdSleepMilliseconds(100);

	sprintf(msgp->data, "\n\n"R2P_MODULE_NAME" module [0x%x 0x%x 0x%x 0x%x 0x%x 0x%x]", uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5]);
	pub.publish(msgp);
	chThdSleepMilliseconds(100);

	adcStart(&ADCD1, NULL);
	adcConvert(&ADCD1, &adcgrp_cfg, adc_buf, 2);
	voltage = adc_buf[0] * 3.3 * 2 / 4095;

	while (!pub.alloc(msgp)) chThdSleepMilliseconds(100);

	if ((voltage > 3.1) && (voltage < 3.5)) {
		sprintf(msgp->data, "Voltage OK (%3.2f V)", voltage);
	} else {
		sprintf(msgp->data, "Voltage FAIL (%3.2f V)", voltage);
	}
	pub.publish(msgp);
	chThdSleepMilliseconds(100);

	spiStart(&SPID1, &spi1cfg);
	extStart(&EXTD1, &extcfg);

	while (!pub.alloc(msgp)) chThdSleepMilliseconds(100);

	// Gyro SPI test
	if (init_l3gd20h(&SPID1) != 0) {
		sprintf(msgp->data, "FAIL - Gyro SPI");
		gyro_fail = true;
	}

	// Gyro data update test
	if (!gyro_fail) {
		gyro_data_t last_data;
		int updates = 0;

		gyroRun(&SPID1, NORMALPRIO + 1);
		chThdSleepMilliseconds(500);

		last_data = gyro_data;

		for (int i = 0; i < 100; i++) {
			if ((gyro_data.x != last_data.x) || (gyro_data.y != last_data.y) || (gyro_data.z != last_data.z)) updates++;
			last_data = gyro_data;
			chThdSleepMilliseconds(10);
		}

		if (updates < 10) {
			sprintf(msgp->data, "FAIL - Gyro data not updating");
			gyro_fail = true;
		}
	}

	// Gyro ok
	if (!gyro_fail) {
		sprintf(msgp->data, "OK   - Gyro");
	}

	pub.publish(msgp);
	chThdSleepMilliseconds(100);

	while (!pub.alloc(msgp)) chThdSleepMilliseconds(100);

	// Accelerometer SPI test
	if (lsm303_init(&SPID1) != 0) {
		sprintf(msgp->data, "FAIL - Accelerometer SPI");
		accmag_fail = true;
	}

	// Accelerometer data update test
	if (!accmag_fail) {
		acc_data_t last_data;
		int updates = 0;

		accRun(&SPID1, NORMALPRIO + 1);
		chThdSleepMilliseconds(100);

		last_data = acc_data;

		for (int i = 0; i < 100; i++) {
			if ((acc_data.x != last_data.x) || (acc_data.y != last_data.y) || (acc_data.z != last_data.z)) updates++;
			last_data = acc_data;
			chThdSleepMilliseconds(10);
		}

		if (updates < 10) {
			sprintf(msgp->data, "FAIL - Accelerometer data not updating");
			accmag_fail = true;
		}
	}

	// Magnetometer data update test
	if (!accmag_fail) {
		mag_data_t last_data;
		int updates = 0;

		magRun(&SPID1, NORMALPRIO + 1);
		chThdSleepMilliseconds(100);

		last_data = mag_data;

		for (int i = 0; i < 100; i++) {
			if ((mag_data.x != last_data.x) || (mag_data.y != last_data.y) || (mag_data.z != last_data.z)) updates++;
			last_data = mag_data;
			chThdSleepMilliseconds(10);
		}

		if (updates < 10) {
			sprintf(msgp->data, "FAIL - Magnetometer data not updating");
			accmag_fail = true;
		}
	}

	// Accelerometer ok
	if (!accmag_fail) {
		sprintf(msgp->data, "OK   - Accelerometer");
	}

	pub.publish(msgp);
	chThdSleepMilliseconds(100);

	while (!pub.alloc(msgp)) chThdSleepMilliseconds(100);

	// Barometer SPI test
	if (init_lps331ap(&SPID1) != 0) {
		sprintf(msgp->data, "FAIL - Barometer SPI");
		accmag_fail = true;
	}
	// Barometer data update test
//	magRun(&SPID1, NORMALPRIO);
//	if (!acc_fail && (update_test(&acc_data, 100) < 10)) {
//		sprintf(msgp->data, "FAIL - Accelerometer data not updating");
//		acc_fail = true;
//	}

	// Barometer ok
	if (!bar_fail) {
		sprintf(msgp->data, "OK   - Barometer");
	}

	pub.publish(msgp);
	chThdSleepMilliseconds(100);

	if (accmag_fail || gyro_fail || bar_fail) {
		for (;;) {
			palTogglePad(LED1_GPIO, LED1);
			chThdSleepMilliseconds(100);
		}
	}

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::ledsub_conf ledsub_conf = { "led" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, test_pub_node, NULL);

	chThdSleepMilliseconds(2000);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 3, madgwick_node, NULL);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}

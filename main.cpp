#include "ch.h"
#include "hal.h"

#include <r2p/Middleware.hpp>
#include <r2p/msg/imu.hpp>
#include <r2p/msg/std_msgs.hpp>

#include <led/nodes/led.hpp>

#include "l3gd20h.h"
#include "lsm303d.h"
#include "lps331ap.h"
#include "madgwick.h"
#include "gps.h"

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(MODULE_NAME, "BOOT_"MODULE_NAME);

/*
 * Madgwick node
 */
extern gyro_data_t gyro_data;
extern acc_data_t acc_data;
extern mag_data_t mag_data;

static const SPIConfig spi1cfg = { 0, 0, 0, SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA , 0};

static const EXTConfig extcfg = { {
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOB, l3gd20h_drdy_callback },
	{ EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, lsm303_int1_cb },
	{ EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, lsm303_int2_cb },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_DISABLED, NULL },
	{ EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, lps331ap_drdy_callback },
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
		MadgwickAHRSupdateIMU((gyro_data.y / 57.143) * 3.141592 / 180.0, (-gyro_data.x / 57.143) * 3.141592 / 180.0,
				(gyro_data.z / 57.143) * 3.141592 / 180.0, acc_data.x / 1000.0, acc_data.y / 1000.0,
				acc_data.z / 1000.0);
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
 * Barometer node
 */
extern baro_data_t baro_data;


msg_t baro_node(void *arg) {
	r2p::Node node("barometer");
	r2p::Publisher<r2p::Float32Msg> baro_pub;
	systime_t time;

	(void) arg;
	chRegSetThreadName("barometer");

	baroRun(&SPID1, NORMALPRIO);

	node.advertise(baro_pub, "barometer");

	time = chTimeNow();

	for (;;) {
		r2p::Float32Msg *msgp;
		if (baro_pub.alloc(msgp)) {
			msgp->data = baro_data.pressure;

			baro_pub.publish(*msgp);
		}

		time += MS2ST(40);
		chThdSleepUntil(time);
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

	r2p::ledpub_conf ledpub_conf = {"led", 1};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledpub_node, &ledpub_conf);

	r2p::ledsub_conf ledsub_conf = { "led" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 3, madgwick_node, NULL);

	r2p::Thread::sleep(r2p::Time::ms(500));

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, baro_node, NULL);

	gps_node_conf gps_node_conf = {"gps_node", "gps"};
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, gps_node, &gps_node_conf);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}


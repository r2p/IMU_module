#include "ch.h"
#include "hal.h"

#include "r2p/Middleware.hpp"
#include <r2p/msg/imu.hpp>
#include <r2p/msg/std_msgs.hpp>

#include "gps.h"
#include "lib/minmea/minmea.h"

static const unsigned char stop_mode[] = "$PMTK161,0*28\r\n";
static const SerialConfig serial_config = {
    9600,
    0,
    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    0
};


msg_t gps_node(void *arg) {
	gps_node_conf * conf = (gps_node_conf *) arg;
	r2p::Node node(conf->name);
	r2p::Publisher<r2p::GPSMsg> gps_pub;
	r2p::GPSMsg *msgp;
	r2p::Publisher<r2p::String64Msg> test_pub;
	r2p::String64Msg * strp;
	systime_t time;
	char line[MINMEA_MAX_LENGTH] = "";
	uint16_t i = 0;

	sdStart(&SD1, &serial_config);
	r2p::Thread::sleep(r2p::Time::ms(500));
//	sdWrite(&SD1, stop_mode, 15);

	node.advertise(gps_pub, conf->topic);
	node.advertise(test_pub, "test");

	while (!chThdShouldTerminate()) {
		if (i < MINMEA_MAX_LENGTH) {
			line[i] = sdGet(&SD1);
		}

		if (line[i] != '\n') {
			i++;
			continue;
		}
#if 0
		if (test_pub.alloc(strp)) {
			strcpy(strp->data, line);
			test_pub.publish(strp);
		}

		i = 0;
		continue;
#endif

		switch (minmea_sentence_id(line, false)) {
			case MINMEA_SENTENCE_RMC: {
				struct minmea_sentence_rmc frame;
				if (minmea_parse_rmc(&frame, line)) {
					if (gps_pub.alloc(msgp)) {
						msgp->valid = frame.valid;
						msgp->satellites = -1;
						msgp->latitude = minmea_tocoord(&frame.latitude);
						msgp->longitude = minmea_tocoord(&frame.longitude);
						gps_pub.publish(msgp);
					}
#if 0
					printf("$RMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
							frame.latitude.value, frame.latitude.scale,
							frame.longitude.value, frame.longitude.scale,
							frame.speed.value, frame.speed.scale);
					printf("$RMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
							minmea_rescale(&frame.latitude, 1000),
							minmea_rescale(&frame.longitude, 1000),
							minmea_rescale(&frame.speed, 1000));
					printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\n",
							minmea_tocoord(&frame.latitude),
							minmea_tocoord(&frame.longitude),
							minmea_tofloat(&frame.speed));
#endif
				}
			} break;

			case MINMEA_SENTENCE_GGA: {
				struct minmea_sentence_gga frame;
				if (minmea_parse_gga(&frame, line)) {
					if (gps_pub.alloc(msgp)) {
						msgp->valid = frame.fix_quality;
						msgp->satellites = frame.satellites_tracked;
						msgp->latitude = minmea_tocoord(&frame.latitude);
						msgp->longitude = minmea_tocoord(&frame.longitude);
						gps_pub.publish(msgp);
					}
#if 0
					printf("$GGA: fix quality: %d\n", frame.fix_quality);
#endif
				}
			} break;

			case MINMEA_SENTENCE_GSV: {
				struct minmea_sentence_gsv frame;
				if (minmea_parse_gsv(&frame, line)) {
					printf("$GSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
					printf("$GSV: sattelites in view: %d\n", frame.total_sats);
#if 0
					for (int i = 0; i < 4; i++)
						printf("$GSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
							frame.sats[i].nr,
							frame.sats[i].elevation,
							frame.sats[i].azimuth,
							frame.sats[i].snr);
#endif
				}
			} break;
		}

		i = 0;
	}
}

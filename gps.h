#pragma once

struct gps_node_conf {
	const char * name;
	const char * topic;
};

msg_t gps_node(void *arg);

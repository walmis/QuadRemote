/*
 * mavHandler.hpp
 *
 *  Created on: Oct 13, 2014
 *      Author: walmis
 */

#ifndef MAVHANDLER_HPP_
#define MAVHANDLER_HPP_

#include <xpcc/architecture.hpp>
#include "mavlink/common/mavlink.h"

class MAVHandler final : xpcc::TickerTask {
public:
	MAVHandler();

	mavlink_gps_raw_int_t gpsStatus;
	mavlink_heartbeat_t heartBeat;
	mavlink_vfr_hud_t vfrHud;
	mavlink_attitude_t attitude;
	mavlink_sys_status_t sysStatus;

	bool getLinkStatus();

protected:
	void handleTick();
	void parseCharFromMAV(uint8_t c);

	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

	mavlink_status_t mavStatus;

	xpcc::Timestamp lastHeartbeat;
};

extern MAVHandler mavHandler;
#endif /* MAVHANDLER_HPP_ */

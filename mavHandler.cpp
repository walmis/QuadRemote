/*
 * mavHandler.cpp
 *
 *  Created on: Oct 13, 2014
 *      Author: walmis
 */

#include <mavHandler.hpp>
#include <remote_control.hpp>

#include "mavlink/common/mavlink.h"

using namespace xpcc;
using namespace xpcc::lpc17;

MAVHandler::MAVHandler() {
}

void MAVHandler::handleTick() {
	uint16_t rx_avail, tx_avail;
	if((rx_avail = btUart.rxAvailable()) && (tx_avail = radio.txAvailable())) {
		while(rx_avail && tx_avail) {
			radio.write(btUart.read());
			rx_avail--;
			tx_avail--;
		}
	}

	if(radio.rxAvailable()) {
		int16_t c;
		while(btUart.txAvailable() && (c = radio.read()) != -1) {
			parseCharFromMAV(c);
			btUart.write(c);
		}
	}
}

void MAVHandler::parseCharFromMAV(uint8_t c) {
	if(mavlink_parse_char(0, c, &msgBuf, &mavStatus)) {
//		printf("Received message with ID %d, sequence: %d from component %d of system %d\n",
//				msgBuf.msgid, msgBuf.seq, msgBuf.compid, msgBuf.sysid);
		switch(msgBuf.msgid) {
		case MAVLINK_MSG_ID_ATTITUDE:
			mavlink_msg_attitude_decode(&msgBuf, &attitude);
			break;
		case MAVLINK_MSG_ID_HEARTBEAT:
			mavlink_msg_heartbeat_decode(&msgBuf, &heartBeat);
			break;
		case MAVLINK_MSG_ID_VFR_HUD:
			mavlink_msg_vfr_hud_decode(&msgBuf, &vfrHud);
			break;
		case MAVLINK_MSG_ID_GPS_RAW_INT:
			mavlink_msg_gps_raw_int_decode(&msgBuf, &gpsStatus);
			break;
		case MAVLINK_MSG_ID_SYS_STATUS:
			mavlink_msg_sys_status_decode(&msgBuf, &sysStatus);
			break;
		}
	}
}

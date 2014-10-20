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

			uint8_t c = btUart.read();

			static mavlink_message_t m;
			if(mavlink_parse_char(0, c, &m, &mavStatus)) {
				printf("Received message with ID %d, sequence: %d from component %d of system %d\n",
					m.msgid, m.seq, m.compid, m.sysid);

				if(m.msgid == MAVLINK_MSG_ID_REQUEST_DATA_STREAM) {
					mavlink_request_data_stream_t d;
					mavlink_msg_request_data_stream_decode(&m, &d);
					printf("%d %d %d %d %d\n", d.req_message_rate, d.req_stream_id, d.start_stop, d.target_component, d.target_system);
				}
			}

			radio.write(c);
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

	static xpcc::Timeout<> heartBeatTimer(1000);

	if(heartBeatTimer.isExpired()) {
		if(radio.txAvailable() >= MAVLINK_MSG_ID_HEARTBEAT_LEN) {

			mavlink_message_t msg;
			size_t len;
			mavlink_msg_heartbeat_pack(255, 102, &msg,
					MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
					MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
			len = mavlink_msg_to_send_buffer(buffer, &msg);

			radio.write(buffer, len);
			heartBeatTimer.restart(1000);
		}
	}
}

void MAVHandler::parseCharFromMAV(uint8_t c) {
	mavlink_message_t msgBuf;
	if(mavlink_parse_char(0, c, &msgBuf, &mavStatus)) {
//		printf("Received message with ID %d, sequence: %d from component %d of system %d\n",
//				msgBuf.msgid, msgBuf.seq, msgBuf.compid, msgBuf.sysid);
		switch(msgBuf.msgid) {
		case MAVLINK_MSG_ID_ATTITUDE:
			mavlink_msg_attitude_decode(&msgBuf, &attitude);
			break;
		case MAVLINK_MSG_ID_HEARTBEAT:
			mavlink_msg_heartbeat_decode(&msgBuf, &heartBeat);

			if(!getLinkStatus()) {
				//comms restored
				//STREAM_EXTRA1
				//STREAM_EXTRA2
				//STREAM_EXTENDED_STATUS

				while(radio.txAvailable() < MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN*3) {
					xpcc::yield();
				}

				uint8_t sysid = msgBuf.sysid;
				printf("requesting data stream from sysid:%d\n", sysid);

				mavlink_message_t msg;
				size_t len;
				mavlink_msg_request_data_stream_pack(255, 0, &msgBuf, sysid,
						0, MAV_DATA_STREAM_EXTRA1, 5, 1);

				len = mavlink_msg_to_send_buffer(buffer, &msgBuf);
				radio.write(buffer, len);

				mavlink_msg_request_data_stream_pack(255, 0, &msgBuf, sysid,
						0, MAV_DATA_STREAM_EXTRA2, 5, 1);
				len = mavlink_msg_to_send_buffer(buffer, &msgBuf);
				radio.write(buffer, len);

				mavlink_msg_request_data_stream_pack(255, 0, &msgBuf, sysid,
						0, MAV_DATA_STREAM_EXTENDED_STATUS, 3, 1);
				len = mavlink_msg_to_send_buffer(buffer, &msgBuf);
				radio.write(buffer, len);
			}

			lastHeartbeat = xpcc::Clock::now();

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

bool MAVHandler::getLinkStatus() {
	return (xpcc::Clock::now() - lastHeartbeat) < 2000;
}

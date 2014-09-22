/*
 * remote_control.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#ifndef REMOTE_CONTROL_HPP_
#define REMOTE_CONTROL_HPP_

#include "radio.hpp"

enum PacketType {
	RC_PACKET,
	RF_PARAM_SET_PACKET
};

class RemoteControl : public Radio {
public:
	struct RCPacket {
		uint8_t id = RC_PACKET;
		uint8_t seq;
		int16_t yawCh;
		int16_t pitchCh;
		int16_t rollCh;
		int8_t throttleCh;
		uint8_t auxCh;
		uint8_t switches;
		uint8_t reserved[3];
	} __attribute__((packed));

	RCPacket rcData;

	uint8_t noiseFloor;

protected:
	void handleInit();
	void handleTick();

	bool sending;

};


#endif /* REMOTE_CONTROL_HPP_ */

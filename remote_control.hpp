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
	PACKET_RC = 100,
	PACKET_RF_PARAM_SET,
	PACKET_DATA_FIRST, //first data fragment
	PACKET_DATA, //data fragment
	PACKET_DATA_LAST //last data fragment
};

struct Packet {
	uint8_t id = PACKET_RC;
	uint8_t seq; //sequence number
	uint8_t ackSeq; //rx acknowledged seq number
} __attribute__((packed));

struct RCPacket : Packet{
	int16_t yawCh;
	int16_t pitchCh;
	int16_t rollCh;
	int16_t throttleCh;
	uint16_t auxCh;
	uint8_t switches;
} __attribute__((packed));

class RemoteControl : public Radio {
public:
	RCPacket rcData;

	uint8_t noiseFloor;

	inline uint32_t getTxBad() {
		return _txBad;
	}

protected:
	void handleInit();
	void handleTick();

	bool sending;

	uint8_t lastSeq;
	uint8_t lastAckSeq;
	uint32_t _txBad;

	uint8_t packetBuf[255];
	uint8_t dataPos;
};


#endif /* REMOTE_CONTROL_HPP_ */

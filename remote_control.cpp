/*
 * remote_control.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#include "remote_control.hpp"
#include <system.hpp>

void RemoteControl::handleInit() {
	Radio::handleInit();

	float freq, afc;
	uint8_t txPow;
	RH_RF22::ModemConfigChoice modemCfg;

	eeprom.get(&EEData::rfFrequency, freq);
	eeprom.get(&EEData::afcPullIn, afc);
	eeprom.get(&EEData::txPower, txPow);
	eeprom.get(&EEData::modemCfg, modemCfg);

	setFrequency(freq, afc);
	setTxPower(txPow);
	setModemConfig(modemCfg);
}

void RemoteControl::handleTick() {
	static PeriodicTimer<> t(20);
	if (!transmitting()) {
		if (sending) {
			sending = false;
			setModeRx();
		}

		if(!sending && available()) {
			uint8_t buf[255];
			uint8_t len = sizeof(buf);

			recv(buf, &len);

			if(len >= sizeof(Packet)) {
				Packet* p = (Packet*)buf;

				switch(p->id) {
				case PACKET_DATA_FIRST:
					dataPos = 0;
					//fall through
				case PACKET_DATA_LAST:
				case PACKET_DATA: {
					uint8_t sz = len-sizeof(Packet);
					if(sz > (sizeof(packetBuf) - dataPos)) {
						sz = sizeof(packetBuf) - dataPos;
					}
					memcpy(packetBuf+dataPos,
							buf+sizeof(Packet), sz);
					dataPos += sz;
				}
				break;

				}
				if(p->id == PACKET_DATA_LAST) {
					printf("received packed len:%d\n", dataPos);
				}

				if(p->id >= PACKET_RC) {
					rcData.ackSeq = p->seq; //acknowledge packet

					//check for lost packets
					if(lastSeq+1 != p->seq) {
						if(p->seq < lastSeq) {
							_rxBad += 255-lastSeq + p->seq;
						} else {
							_rxBad += p->seq - lastSeq;
						}
					}

					lastSeq = p->seq;
					lastAckSeq = p->ackSeq;
				}

			}

			//XPCC_LOG_DEBUG.dump_buffer(buf, len);
		}
		if (!sending && t.isExpired()) {
			//sent packet was not acknowledged

			if(std::abs(rcData.seq - lastAckSeq) > 1) {
				_txBad++;
			}

			rcData.seq++;
			rcData.pitchCh = axes.getChannel(AXIS_PITCH_CH);
			rcData.rollCh = axes.getChannel(AXIS_ROLL_CH);
			rcData.yawCh = axes.getChannel(AXIS_YAW_CH);
			rcData.rollCh = axes.getChannel(AXIS_ROLL_CH);
			rcData.throttleCh = axes.getChannel(AXIS_THROTTLE_CH);
			rcData.auxCh = axes.getChannel(AXIS_AUX6_CH);

			noiseFloor = ((uint16_t) noiseFloor * 5 + rssiRead()) / 6;

			send((uint8_t*) (&rcData), sizeof(rcData));

			sending = true;

		}
	}

}


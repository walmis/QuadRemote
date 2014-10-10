/*
 * remote_control.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#include "remote_control.hpp"
#include <system.hpp>
#include "radio.hpp"
#include <Axes.hpp>

void RemoteControl::handleInit() {
	Radio::handleInit();

	eeprom.get(&EEData::rfFrequency, freq);
	eeprom.get(&EEData::afcPullIn, afcPullIn);
	eeprom.get(&EEData::txPower, txPower);
	eeprom.get(&EEData::modemCfg, modemCfg);
	eeprom.get(&EEData::txInterval, txInterval);
	eeprom.get(&EEData::fhChannels, numFhChannels);

	txPacketTimer.restart(txInterval);

	setFHStepSize(10);
	setFrequency(freq, afcPullIn);
	setTxPower(txPower);
	setModemConfig(modemCfg);
}

uint8_t RemoteControl::getLinkQuality() {
	if(Clock::now() - radio.getLastPreambleTime() > 250) {
		return 0;
	}
	int percent = 2 * (radio.lastRssi() + 100);
	if(percent > 100) percent = 100;
	if(percent < 0) percent = 0;
	return percent;
}
static ProfileTimer t;

void RemoteControl::handleTxComplete() {
	if(numFhChannels)
		setFHChannel((rcData.seq^0x55) % numFhChannels);

	setModeRx();
}

void RemoteControl::handleRxComplete() {

}

void RemoteControl::handleTick() {

	if (!transmitting()) {

		if (sending) {
			sending = false;

			//radio data is sent, now we can apply new settings
			if(newRadioDataSent) {
				setFrequency(freq, afcPullIn);
				setModemConfig(modemCfg);
				newRadioDataSent = false;
			}
		}


		if(available()) {
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
					if(p->id == PACKET_DATA_LAST) {
						dataLen = dataPos;
						dataPos = 0;
					}
				}
				break;

				}
				if(p->id == PACKET_DATA_LAST) {
					printf("received packed len:%d\n", dataLen);
				}

				if(p->id >= PACKET_RC) {
					//printf("rx seq %d ackseq %d\n", p->seq, p->ackSeq);
					//check for lost packets
					if((uint8_t)(lastSeq+1) != p->seq) {
						if(p->seq < lastSeq) {
							_rxBad += 255-lastSeq + p->seq;
						} else {
							_rxBad += p->seq - (lastSeq+1);
						}
					}

					rcData.ackSeq = p->seq; //acknowledge packet
					lastSeq = p->seq;
					lastAckSeq = p->ackSeq;
				}

			}

			//XPCC_LOG_DEBUG.dump_buffer(buf, len);
		} else


		if (!sending && txPacketTimer.isExpired() && rssiRead() < 80) {
			//sent packet was not acknowledged
			if(rcData.seq != lastAckSeq) {
				//if(rcData.seq == lastAckSeq)
				//if packet was lost, restore previous FH channel
				//if(numFhChannels)
				//	setFHChannel(((rcData.seq-1)^0x55) % numFhChannels);

				_txBad++;
			}

			rcData.seq++;
			rcData.pitchCh = axes.getChannel(AXIS_PITCH_CH);
			rcData.rollCh = axes.getChannel(AXIS_ROLL_CH);
			rcData.yawCh = axes.getChannel(AXIS_YAW_CH);
			rcData.rollCh = axes.getChannel(AXIS_ROLL_CH);
			rcData.throttleCh = axes.getChannel(AXIS_THROTTLE_CH);
			rcData.auxCh = axes.getChannel(AXIS_AUX6_CH);
			//rcData.switches = (auxSw5::read() & 0x01);

			noiseFloor = ((uint16_t) noiseFloor * 5 + rssiRead()) / 6;

			if(configurationChanged) {
				RadioCfgPacket p;
				p.seq = rcData.seq;
				p.ackSeq = rcData.ackSeq;
				p.frequency = getFreq();
				p.afcPullIn = getAfcPullIn();
				p.fhChannels = getNumFhChannels();
				p.modemCfg = getModemCfg();
				p.txPower = getTxPower();

				send((uint8_t*) (&p), sizeof(RadioCfgPacket));
				newRadioDataSent = true;
				configurationChanged = false;
			} else {
				send((uint8_t*) (&rcData), sizeof(rcData));
			}

			txPacketTimer.restart(txInterval);
			//t.end();
			//t.start();
			sending = true;

		}

	}

}


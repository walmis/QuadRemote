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
#include <Switches.h>

void RemoteControl::handleInit() {
	Radio::handleInit();

	eeprom.get(&EEData::rfFrequency, freq);
	eeprom.get(&EEData::afcPullIn, afcPullIn);
	eeprom.get(&EEData::txPower, txPower);
	eeprom.get(&EEData::modemCfg, modemCfg);
	eeprom.get(&EEData::txInterval, txInterval);
	eeprom.get(&EEData::fhChannels, numFhChannels);

	txPacketTimer.restart(txInterval);

	Radio::setFHStepSize(10);
	Radio::setFrequency(freq/1000.0, afcPullIn);
	Radio::setTxPower(txPower);
	Radio::setModemConfig(modemCfg);
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
				setMode(RHMode::RHModeIdle);

				Radio::setFrequency(freq/1000.0, afcPullIn);
				Radio::setModemConfig(modemCfg);
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

				case PACKET_DATA: {
					uint8_t size = len - sizeof(Packet);
					printf("recv %d\n", size);
					if(rxbuf.bytes_free() < size) {
						printf("drop packet\n");
					} else {
						rxbuf.write(buf+sizeof(Packet), size);
					}
				}
				break;

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
			bool noAck = false;
			//sent packet was not acknowledged
			if(rcData.seq != lastAckSeq) {
				//if(rcData.seq == lastAckSeq)
				//if packet was lost, restore previous FH channel
				//if(numFhChannels)
				//	setFHChannel(((rcData.seq-1)^0x55) % numFhChannels);
				noAck = true;
				_txBad++;
			}

			rcData.seq++;
			rcData.pitchCh = axes.getChannel(AXIS_PITCH_CH);
			rcData.rollCh = axes.getChannel(AXIS_ROLL_CH);
			rcData.yawCh = axes.getChannel(AXIS_YAW_CH);
			rcData.rollCh = axes.getChannel(AXIS_ROLL_CH);
			rcData.throttleCh = axes.getChannel(AXIS_THROTTLE_CH);
			rcData.auxCh = axes.getChannel(AXIS_AUX6_CH);
			rcData.switches = switches.getBitmask();

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
				printf("sending radio conf\n");
				newRadioDataSent = true;
				configurationChanged = false;
			} else {
				if(noAck && dataLen) {
					printf("retry\n");
					//retransmit previous data
					memcpy(packetBuf, (void*)&rcData, sizeof(RCPacket));
					send(packetBuf, dataLen);

				} else {
					size_t avail = txbuf.bytes_used();
					if(avail) {
						size_t space = sizeof(packetBuf) - sizeof(RCPacket);
						uint8_t* ptr = packetBuf + sizeof(RCPacket);
						//copy pending rc packet to tx buffer
						memcpy(packetBuf, (void*)&rcData, sizeof(RCPacket));
						while(space && avail) {
							*ptr++ = txbuf.read();
							space--;
							avail--;
						}
						dataLen = ptr - packetBuf;

						printf("send %d\n", dataLen-sizeof(RCPacket));
						send(packetBuf, dataLen);

					} else {
						dataLen = 0;

						send((uint8_t*) (&rcData), sizeof(rcData));
					}

				}
			}
			txPacketTimer.restart(txInterval);
			sending = true;
		}

	}

}


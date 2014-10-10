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

struct RadioCfgPacket : Packet {
	RadioCfgPacket() {
		id = PACKET_RF_PARAM_SET;
	}
	float frequency;
	float afcPullIn;
	uint8_t modemCfg;
	uint8_t fhChannels;
	uint8_t txPower;
} __attribute__((packed));

struct RCPacket : Packet{
	int16_t yawCh;
	int16_t pitchCh;
	int16_t rollCh;
	int16_t throttleCh;
	int16_t auxCh;
	uint8_t switches;
} __attribute__((packed));

class RemoteControl : public Radio {
public:
	RemoteControl() : txPacketTimer(20) {}

	RCPacket rcData;

	uint8_t getLinkQuality();

	inline uint32_t getTxBad() {
		return _txBad;
	}

	float getAfcPullIn() const {
		return afcPullIn;
	}

	void setAfcPullIn(float afcPullIn) {
		configurationChanged = true;
		this->afcPullIn = afcPullIn;
		eeprom.put(&EEData::afcPullIn, afcPullIn);
	}

	float getFreq() const {
		return freq;
	}

	void setFreq(float freq) {
		configurationChanged = true;
		eeprom.put(&EEData::rfFrequency, freq);
		this->freq = freq;
	}

	RH_RF22::ModemConfigChoice getModemCfg() const {
		return modemCfg;
	}

	void setModemCfg(RH_RF22::ModemConfigChoice modemCfg) {
		configurationChanged = true;
		eeprom.put(&EEData::modemCfg, modemCfg);
		this->modemCfg = modemCfg;
	}

	uint8_t getNumFhChannels() const {
		return numFhChannels;
	}

	void setNumFhChannels(uint8_t numFhChannels) {
		configurationChanged = true;
		eeprom.put(&EEData::fhChannels, numFhChannels);
		this->numFhChannels = numFhChannels;
	}

	uint8_t getTxInterval() const {
		return txInterval;
	}

	void setTxInterval(uint8_t txInterval) {
		eeprom.put(&EEData::txInterval, txInterval);
		txPacketTimer.restart(txInterval);
		this->txInterval = txInterval;
	}

	uint8_t getTxPower() const {
		return txPower;
	}

	void setTxPower(uint8_t txPower) {
		configurationChanged = true;
		this->txPower = txPower;
		eeprom.put(&EEData::txPower, txPower);
		RH_RF22::setTxPower(txPower);
	}

	uint8_t getNoiseFloor() {
		return noiseFloor;
	}

	int8_t getRssi() {
		rssi = (3*rssi + (int)lastRssi()) / 4;
		return rssi;
	}

protected:
	friend class FreqConf;

	void handleTxComplete();
	void handleRxComplete();

	uint8_t noiseFloor;
	int8_t rssi;

	void handleInit();
	void handleTick();

	bool sending;

	uint8_t lastSeq;
	uint8_t lastAckSeq;
	uint32_t _txBad;

	uint8_t packetBuf[255];
	uint8_t dataPos;
	uint8_t dataLen;

	Timeout<> txPacketTimer;

	///configuration parameters////
	float freq;
	float afcPullIn;
	uint8_t txPower;
	uint8_t txInterval;
	RH_RF22::ModemConfigChoice modemCfg;
	uint8_t numFhChannels; //Frequency hopping channels

	bool configurationChanged;
	bool newRadioDataSent;
	////////
};

extern RemoteControl radio;

#endif /* REMOTE_CONTROL_HPP_ */

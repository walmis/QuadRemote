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
	if (t.isExpired()) {
		if (!transmitting()) {

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
		} else {

		}
	}
	if (sending && !transmitting()) {
		sending = false;
		setModeRx();
	}
}


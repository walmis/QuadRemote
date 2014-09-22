/*
 * eedata.hpp
 *
 *  Created on: May 8, 2014
 *      Author: walmis
 */

#ifndef EEDATA_HPP_
#define EEDATA_HPP_

#include "eeprom.hpp"
#include <xpcc/math.hpp>
#include <RH_RF22.h>

struct EEData {
	uint8_t token;

	float rfFrequency; //radio frequency
	float afcPullIn;
	uint8_t txPower;
	RH_RF22::ModemConfigChoice modemCfg;

	uint16_t axisCalibration[5][2];

} __attribute((packed));

const EEData eeDefaults = {
		TOKEN,
		429.0f,
		0.05f,
		RH_RF22_TXPOW_1DBM,
		RH_RF22::GFSK_Rb57_6Fd28_8,
		{{0, 4096}, {0, 4096}, {0, 4096}, {0, 4096}, {0, 4096}}

};

#endif /* EEDATA_HPP_ */

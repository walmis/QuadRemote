/*
 * eedata.hpp
 *
 *  Created on: May 8, 2014
 *      Author: walmis
 */

#ifndef EEDATA_HPP_
#define EEDATA_HPP_

#include "eeprom.hpp"

struct CalibrationData {
	CalibrationData() : min(0), max(0) {};

	uint16_t min;
	uint16_t max;
};

struct EEData {
	uint8_t token = TOKEN;

	CalibrationData PWMInputCalibration[5];

	xpcc::Quaternion<float> qTrim;
	xpcc::Quaternion<float> qRotationOffset;

	float ratePIDParams[3];
	float PIDParams[3];
	float heightPIDparams[4]; //p, i, d, maxoutput

	float yawGain;
};


#endif /* EEDATA_HPP_ */

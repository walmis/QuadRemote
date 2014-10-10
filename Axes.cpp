/*
 * Axes.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#include "Axes.hpp"

void Axes::handleInit() {
	calibrating = false;
	Pinsel::setFunc(0, 3, 2);
	Pinsel::setFunc(0, 25, 1);
	Pinsel::setFunc(0, 26, 1);
	Pinsel::setFunc(1, 30, 3);
	Pinsel::setFunc(1, 31, 3);
	ADC::enableChannel(2);
	ADC::enableChannel(3);
	ADC::enableChannel(4);
	ADC::enableChannel(5);
	ADC::enableChannel(6);

	if (!eeprom.get(&EEData::axisCalibration, calData)) {
		panic("eeprom cal read fail");
	}

}

void Axes::handleTick() {
	static PeriodicTimer<> t(1);
	if (t.isExpired()) {

		for (int i = 0; i < 5; i++) {
			int16_t adc = ADC::getData(i + 2);

			channels[i] = (channels[i] * 4 - channels[i] + adc + (4 / 2)) / 4;

			if (i == AXIS_ROLL_CH) {
				//invert roll channel
				channels[i] = 4096 - channels[i];
			}

			if (calibrating) {
				if (channels[i] > calData[i][1]) {
					calData[i][1] = channels[i];
				}
				if (channels[i] < calData[i][0]) {
					calData[i][0] = channels[i];
				}
			}
		}
	}
}

void Axes::calibration(bool en) {
	calibrating = en;
	if (en) {
		for (int i = 0; i < 5; i++) {
			calData[i][0] = 0xFFFF; //min
			calData[i][1] = 0; //max
		}
	} else {
		if(!eeprom.put(&EEData::axisCalibration, calData)) {
			panic("eeprom write failed");
		}
	}
}

int16_t Axes::getChannel(uint8_t ch) {
	uint16_t min = calData[ch][0];
	uint16_t max = calData[ch][1];
	if(max == min) return -4096;
	return (channels[ch] - min) * 1024 / (max - min);
}

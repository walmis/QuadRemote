/*
 * battery.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#include "battery.hpp"

/* voltages (millivolt) of 0%, 10%, ... 100% */
const static unsigned short percent_to_volt_discharge[2][11] = {
/* measured values */
{ 2800 / 3, 3250 / 3, 3410 / 3, 3530 / 3, 3640 / 3, 3740 / 3, 3850 / 3, 3950
		/ 3, 4090 / 3, 4270 / 3, 4750 / 3 }, /* Alkaline */
{ 3100 / 3, 3550 / 3, 3630 / 3, 3690 / 3, 3720 / 3, 3740 / 3, 3760 / 3, 3780
		/ 3, 3800 / 3, 3860 / 3, 4050 / 3 } /* NiMH */
};

void Battery::handleTick() {
	static PeriodicTimer<> t(50);
	if(t.isExpired()) {
		int16_t adc = ADC::getData(AD_CH_VBAT);
		//4096 - 3.3V
		//30k 8.2k
		float bat = (adc / (4096 / 3.3f)) * 4.651f;
		if (std::abs(bat - packVoltage) > 0.2f) {
			packVoltage = bat;
		} else {
			packVoltage = (packVoltage * 50 + bat) / 51;
		}
		cellVoltage = packVoltage / 8;
	}
}

int Battery::getBatteryPercent() {
	return voltage_to_percent(cellVoltage * 1000, percent_to_volt_discharge[1]);
}

bool Battery::onUSBPower() {
	if (packVoltage < 4.5 && packVoltage > 4.1) {
		return true;
	}
	return false;
}

int Battery::voltage_to_percent(int voltage,
		const unsigned short * table) {
	if (voltage <= table[0]) {
		return 0;
	} else if (voltage >= table[10]) {
		return 100;
	} else {
		/* search nearest value */
		int i = 0;
		while (i < 10 && table[i + 1] < voltage)
			i++;
		/* interpolate linear between the smaller and greater value */
		/* Tens digit, 10% per entry, ones digit: interpolated */
		return i * 10 + (voltage - table[i]) * 10 / (table[i + 1] - table[i]);
	}
}



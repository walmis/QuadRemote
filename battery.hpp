/*
 * battery.hpp
 *
 *  Created on: Sep 19, 2014
 *      Author: walmis
 */

#ifndef BATTERY_HPP_
#define BATTERY_HPP_

#include <xpcc/architecture.hpp>
#include "pindefs.hpp"

using namespace xpcc;
using namespace xpcc::lpc17;


class Battery {
public:
	/* voltages (millivolt) of 0%, 10%, ... 100% */
	const unsigned short percent_to_volt_discharge[2][11] = {
	/* measured values */
	{ 2800 / 3, 3250 / 3, 3410 / 3, 3530 / 3, 3640 / 3, 3740 / 3, 3850 / 3, 3950
			/ 3, 4090 / 3, 4270 / 3, 4750 / 3 }, /* Alkaline */
	{ 3100 / 3, 3550 / 3, 3630 / 3, 3690 / 3, 3720 / 3, 3740 / 3, 3760 / 3, 3780
			/ 3, 3800 / 3, 3860 / 3, 4050 / 3 } /* NiMH */
	};

	void update() {

		int16_t adc = ADC::getData(AD_CH_VBAT);

		//4096 - 3.3V
		//30k 8.2k

		float bat = (adc / (4096 / 3.3f)) * 4.651f;

		if(fabsf(bat-packVoltage) > 0.2f) {
			packVoltage = bat;
		} else {
			packVoltage = (packVoltage*50 + bat) / 51;
		}

		cellVoltage =  packVoltage / 8;

	}

	int getBatteryPercent() {
		return voltage_to_percent(cellVoltage*1000,
				percent_to_volt_discharge[1]);
	}

	bool onUSBPower() {
		if(packVoltage < 4.5 && packVoltage > 4.1) {
			return true;
		}
		return false;
	}

	float getPackVoltage(){
		return packVoltage;
	}

	float getCellVoltage(){
		return cellVoltage;
	}

private:
	float cellVoltage;
	float packVoltage;

	/* look into the percent_to_volt_* table and get a realistic battery level */
	int voltage_to_percent(int voltage, const unsigned short* table) {
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
			return i * 10
					+ (voltage - table[i]) * 10 / (table[i + 1] - table[i]);
		}
	}
};



#endif /* BATTERY_HPP_ */

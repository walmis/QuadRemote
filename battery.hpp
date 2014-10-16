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
#include <cmath>


class Battery final : xpcc::TickerTask {
public:
	int getBatteryPercent();

	bool onUSBPower();

	float getPackVoltage(){
		return packVoltage;
	}

	float getCellVoltage(){
		return cellVoltage;
	}

private:

	void handleTick();

	float cellVoltage;
	float packVoltage;

	/* look into the percent_to_volt_* table and get a realistic battery level */
	int voltage_to_percent(int voltage, const unsigned short * table);
};

extern Battery battery;

#endif /* BATTERY_HPP_ */

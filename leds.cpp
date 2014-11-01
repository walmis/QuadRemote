/*
 * Leds.cpp
 *
 *  Created on: Sep 24, 2014
 *      Author: walmis
 */

#include <system.hpp>
#include <eeprom/eeprom.hpp>
#include <battery.hpp>
#include <remote_control.hpp>
#include <leds.hpp>
#include <battery.hpp>
#include "LiPo_Bat.hpp"
#include <mavHandler.hpp>

Leds::Mode Leds::getMode() {
	return mode;
}

void Leds::setMode(Leds::Mode mode) {
	this->mode = mode;
	eeprom.put(&EEData::ledMode, mode);
}

void Leds::handleInit() {
	eeprom.get(&EEData::ledMode, mode);
}

void Leds::handleTick() {
	static PeriodicTimer<> t(10);

	if (t.isExpired()) {
		switch (mode) {
		case Leds::Mode::MODE_BATTERY:
			if (battery.onUSBPower()) {
				ledGreen::set();
				ledRed::set();
				ledYellow::set();
			} else {
				uint8_t percent = battery.getBatteryPercent();
				if (percent < 50) {
					ledGreen::reset();
				} else {
					ledGreen::set();
				}
				if (percent < 25) {
					ledYellow::reset();
				} else {
					ledYellow::set();
				}

				ledRed::set();
			}
			break;
		case Leds::Mode::MODE_SIGNAL_Q: {
			uint8_t q = radio.getLinkQuality();
			if (q < 50) {
				ledGreen::reset();
			} else {
				ledGreen::set();
			}
			if (q < 25) {
				ledYellow::reset();
			} else {
				ledYellow::set();
			}
			if (q < 5) {
				ledRed::reset();
			} else {
				ledRed::set();
			}
		}
			break;
		case Leds::Mode::MODE_REMOTE_BATTERY:
			int8_t percent;
			uint16_t voltage = mavHandler.sysStatus.voltage_battery;
			uint8_t cells;

			if(voltage == 0) break;

			if(voltage > 10500 && voltage < 12700) {
				cells = 3;
			} else if(voltage > 12700) {
				cells = 4;
			} else {
				cells = 2;
			}

			percent = LiPo_Bat::getPercent(voltage / cells);
			percent = std::min(percent, mavHandler.sysStatus.battery_remaining);

			if (percent < 75) {
				ledGreen::reset();
			} else {
				ledGreen::set();
			}
			if (percent < 35) {
				ledYellow::reset();
			} else {
				ledYellow::set();
			}
			if (percent < 15) {
				ledRed::reset();
			} else {
				ledRed::set();
			}

			break;

		}
	}
}

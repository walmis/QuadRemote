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

	if(t.isExpired()) {
		if(mode == Leds::Mode::MODE_BATTERY) {
			if(battery.onUSBPower()) {
				ledGreen::set();
				ledRed::set();
				ledYellow::set();
			} else {
				uint8_t percent = battery.getBatteryPercent();
				if(percent < 50) {
					ledGreen::reset();
				} else {
					ledGreen::set();
				}
				if(percent < 25) {
					ledYellow::reset();
				} else {
					ledYellow::set();
				}

				ledRed::set();
			}

		} else if(mode == Leds::Mode::MODE_SIGNAL_Q) {

			uint8_t q = radio.getLinkQuality();

			if(q < 50) {

				ledGreen::reset();
			} else {
				ledGreen::set();
			}
			if(q < 25) {
				ledYellow::reset();
			} else {
				ledYellow::set();
			}
			if(q < 5) {
				ledRed::reset();
			} else {
				ledRed::set();
			}

		} else if(mode == Leds::Mode::MODE_TXRX) {

		}

	}

}

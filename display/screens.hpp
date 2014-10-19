/*
 * screens.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#ifndef SCREENS_HPP_
#define SCREENS_HPP_

#include "info_screen.hpp"
#include "configurators.hpp"
#include "Switches.h"
#include "mavHandler.hpp"

class RadioScreen : public InfoScreen {
public:
	static bool populate(LCDLine* lines, uint8_t nLines) {
		int rssi = radio.getRssi();
		int noise = (radio.getNoiseFloor() * 100 / 190) - 127;
		//int noise = ((int)radio.rssiRead()*100 / 190) - 127;

		int rxe = radio.getRxBad();
		int txe = radio.getTxBad();

		txe %= 100;
		rxe %= 100;
		int tx = radio.getTxGood() % 1000;
		int rx = radio.getRxGood() % 1000;

		lines[0].addField("T:");
		lines[0].addField(tx, 3);
		lines[0].addField("/");
		lines[0].addField(txe, 2);

		lines[0].addField(" dBm");
		lines[0].addField(rssi, 4);

		lines[1].addField("R:");
		lines[1].addField(rx, 3);
		lines[1].addField("/");
		lines[1].addField(rxe, 2);

		lines[1].addField(noise, 8);
		return true;
	}

	static Configurator* getConfigurator(uint8_t i) {
		static FreqConf c;

		if(i < sizeof(c.confIndex)/sizeof(c.confIndex[0])) {
			return &c;
		}

		return 0;
	}
};

class BatScreen : public InfoScreen {
public:
	static bool populate(LCDLine* lines, uint8_t nLines) {
		lines[0].addField("SYS");

		if(!battery.onUSBPower()) {
			lines[1].addField(battery.getBatteryPercent(), 3);
			lines[1].addField("% ");

			lines[1].addField(battery.getPackVoltage(), 4);
			lines[1].addField('V');
		} else {
			lines[1].addField("VUSB");
		}
		return true;
	}
	static Configurator* getConfigurator(uint8_t i) {
		static LedConf c;
		if(i == 0) return &c;

		return 0;
	}
};

class AxesScreen : public InfoScreen {
public:
	static bool populate(LCDLine* lines, uint8_t nLines) {
		lines[0].addField(axes.getChannel(0), 5);
		lines[0].addField(axes.getChannel(1), 5);
		lines[0].addField(axes.getChannel(2), 5);

		lines[1].addField(axes.getChannel(3), 5);
		lines[1].addField(axes.getChannel(4), 5);

		lines[1].addField(switches.getState(0), 2);
		lines[1].addField(switches.getState(1), 1);
		lines[1].addField(switches.getState(2), 1);
		lines[1].addField(switches.getState(3), 1);
		lines[1].addField(switches.getState(4), 1);
		return true;
	}

	static Configurator* getConfigurator(uint8_t i) {
		static CalConf c;
		if(i == 0)
			return &c;

		return 0;
	}
};

class GpsStatus : public InfoScreen {
public:
	static bool populate(LCDLine* lines, uint8_t nLines) {
		char* gpsFix;
		switch(mavHandler.gpsStatus.fix_type) {
		case 0:
			gpsFix = "NOHW";
			break;
		case 1:
			gpsFix = "NOFIX";
			break;
		case 2:
			gpsFix = "2DFIX";
			break;
		default:
			gpsFix = "3DFIX";
		}
		lines[0] << "GPS:";
		lines[0].addField(gpsFix);
		lines[0] << " SAT:";
		lines[0] << mavHandler.gpsStatus.satellites_visible;

		lines[1] << "SPD:";
		lines[1].printf("%.1f", 4, true, mavHandler.vfrHud.groundspeed*3.6f);

		lines[1] << " ALT:";
		lines[1].addField(lroundf(mavHandler.gpsStatus.alt/1000.0), 3);
		return true;
	}

	static Configurator* getConfigurator(uint8_t i) {
		return 0;
	}
};

class AttitudeDisplay : public InfoScreen {
public:
	static bool populate(LCDLine* lines, uint8_t nLines) {
		lines[0] << "HDG:";
		lines[0].addField(mavHandler.vfrHud.heading, 0);

		lines[0] << "\337ALT:";
		lines[0].addField(mavHandler.vfrHud.alt, 0);

		lines[1] << "P:";
		lines[1].addField(lroundf(mavHandler.attitude.pitch*(180.0/M_PI)), 3);
		lines[1] << "\337";

		lines[1] << " R:";
		lines[1].addField(lroundf(mavHandler.attitude.roll*(180.0/M_PI)), 3);
		lines[1] << "\337";
		return true;
	}

	static Configurator* getConfigurator(uint8_t i) {
		return 0;
	}
};

const char* modes[] = {"STABILIZE", "ACRO", "ALTHOLD", "AUTO", "GUIDED", "LOITER",
		"RTL", "CIRCLE", "UNK", "LAND", "OFLOITER", "DRIFT", "SPORT", "SPORT", "FLIP", "AUTOTUNE", "POSHOLD"};

class MavStatusDisplay : public InfoScreen {
public:
	static bool populate(LCDLine* lines, uint8_t nLines) {
		const char* mode = "UNKNOWN";
		if(mavHandler.heartBeat.custom_mode < sizeof(modes)) {
			mode = modes[mavHandler.heartBeat.custom_mode];
		}

		lines[0].addField(mode, 10, false);

		lines[0] << "BAT:";
		lines[0] << mavHandler.sysStatus.battery_remaining;

		if(mavHandler.heartBeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
			lines[1] << "ARMD ";
		} else {
			lines[1] << "SAFE ";
		}

		lines[1].printf("%.1fV ", 6, 0, mavHandler.sysStatus.voltage_battery/1000.0f);

		lines[1].printf("%.1fA", 5, 1, mavHandler.sysStatus.current_battery/100.0f);
		return true;
	}

	static Configurator* getConfigurator(uint8_t i) {
		return 0;
	}
};

#endif /* SCREENS_HPP_ */

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

class RadioScreen : public InfoScreen {
public:
	void populate(LCDLine* lines, uint8_t nLines) {
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
	}

	Configurator* getConfigurator(uint8_t i) {
		static FreqConf c;

		if(i < sizeof(c.confIndex)/sizeof(c.confIndex[0])) {
			return &c;
		}

		return 0;
	}
};

class BatScreen : public InfoScreen {
public:
	void populate(LCDLine* lines, uint8_t nLines) {
		lines[0].addField("SYS");

		if(!battery.onUSBPower()) {
			lines[1].addField(battery.getBatteryPercent(), 3);
			lines[1].addField("% ");

			lines[1].addField(battery.getPackVoltage(), 4);
			lines[1].addField('V');
		} else {
			lines[1].addField("VUSB");
		}
	}
	Configurator* getConfigurator(uint8_t i) {
		static LedConf c;
		if(i == 0) return &c;

		return 0;
	}
};

class AxesScreen : public InfoScreen {
public:
	void populate(LCDLine* lines, uint8_t nLines) {
		lines[0].addField(axes.getChannel(0), 5);
		lines[0].addField(axes.getChannel(1), 5);
		lines[0].addField(axes.getChannel(2), 5);

		lines[1].addField(axes.getChannel(3), 5);
		lines[1].addField(axes.getChannel(4), 5);

		lines[1].addField(btnL::read(), 2);
		lines[1].addField(btnR::read(), 1);
		lines[1].addField(btnU::read(), 1);
		lines[1].addField(btnD::read(), 1);
		lines[1].addField(auxSw5::read(), 1);
	}

	Configurator* getConfigurator(uint8_t i) {
		static CalConf c;
		if(i == 0)
			return &c;

		return 0;
	}
};


#endif /* SCREENS_HPP_ */

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
		int rssi = radio.lastRssi();
		int noise = (radio.noiseFloor *100 / 190) - 127;
		//int noise = ((int)radio.rssiRead()*100 / 190) - 127;

		int rxe = radio.getRxBad();
		static int txe = 0;

		txe %= 100;
		int tx = radio.getTxGood() % 1000;
		int rx = radio.getRxGood() % 1000;

		lines[0].addTextField("T:");
		lines[0].addIntegerField(tx, 3);
		lines[0].addTextField("/");
		lines[0].addIntegerField(txe, 2);

		lines[0].addTextField(" dBm");
		lines[0].addIntegerField(rssi, 4);

		lines[1].addTextField("R:");
		lines[1].addIntegerField(rx, 3);
		lines[1].addTextField("/");
		lines[1].addIntegerField(rxe, 2);

		lines[1].addIntegerField(noise, 8);
	}

	Configurator* getConfigurator(uint8_t i) {
		static FreqConf c;
		static ModemConf c2;
		static TestConf c3;

		switch(i) {
		case 0:
			return &c;
		case 1:
			return &c2;
		//case 2:
			//return &c3;
		}

		return 0;
	}
};

class BatScreen : public InfoScreen {
public:
	void populate(LCDLine* lines, uint8_t nLines) {
		battery.update();

		lines[0].addTextField("SYS");

		if(!battery.onUSBPower()) {
			lines[1].addField(battery.getBatteryPercent(), 3);
			lines[1].addField("% ");

			lines[1].addField(battery.getPackVoltage(), 4);
			lines[1].addField('V');
		} else {
			lines[1].addField("VUSB");
		}
	}
};

class AxesScreen : public InfoScreen {
public:
	void populate(LCDLine* lines, uint8_t nLines) {
		lines[0].addIntegerField(axes.getChannel(0), 5);
		lines[0].addIntegerField(axes.getChannel(1), 5);
		lines[0].addIntegerField(axes.getChannel(2), 5);

		lines[1].addIntegerField(axes.getChannel(3), 5);
		lines[1].addIntegerField(axes.getChannel(4), 5);

		lines[1].addIntegerField(btnL::read(), 2);
		lines[1].addIntegerField(btnR::read(), 1);
		lines[1].addIntegerField(btnU::read(), 1);
		lines[1].addIntegerField(btnD::read(), 1);
		lines[1].addIntegerField(auxSw5::read(), 1);
	}

	Configurator* getConfigurator(uint8_t i) {
		static CalConf c;
		if(i == 0)
			return &c;

		return 0;
	}
};


#endif /* SCREENS_HPP_ */

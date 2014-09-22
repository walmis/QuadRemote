/*
 * configurators.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#ifndef CONFIGURATORS_HPP_
#define CONFIGURATORS_HPP_

#include "configurators.hpp"

class FreqConf : public Configurator {
public:
	float f = 433.0;
	void populate(LCDLine* lines, uint8_t nLines) {
		lines[0] << "Frequency:";
		lines[1] << f;
	}

	void nextValue() {
		f+=0.1;
	}
	void prevValue() {
		f-=0.1;
	}
};

class ModemConf : public Configurator {
public:
	float f = 433.0;

	void populate(LCDLine* lines, uint8_t nLines) {
		lines[0] << "Modem cfg:";
		lines[1] << f;
	}

	void nextValue() {
		f+=0.1;
	}
	void prevValue() {
		f-=0.1;
	}
};

class TestConf : public Configurator {
public:
	void populate(LCDLine* lines, uint8_t nLines) {
		//lines[0] << "Modem cfg:";
		//lines[1] << f;
	}

	void nextValue() {
	}
	void prevValue() {
	}
};

class CalConf : public Configurator {
public:
	uint8_t i = 0;
	void populate(LCDLine* lines, uint8_t nLines) {
		if(i == 0) {
			lines[0] << "Calibration";
			lines[1] << "Press DN";
		}
		else if(i == 1) {
			lines[0] << "Press DN and";
			lines[1] << "Move all axes";
			axes.calibration(true);
		}
		else if(i == 2) {
			lines[0] << "Cal ";
			lines[0].addIntegerField(axes.getChannel(0), 5);
			lines[0].addIntegerField(axes.getChannel(1), 5);

			lines[1].addIntegerField(axes.getChannel(2), 5);
			lines[1].addIntegerField(axes.getChannel(3), 5);
			lines[1].addIntegerField(axes.getChannel(4), 5);
		}
		else if(i == 3) {
			lines[0] << "Done. Press DN";
			lines[1] << "to finish";
		}
		else if(i == 4) {
			axes.calibration(false);
			i = 0;
		}
	}

	void nextValue() { //up
		if(i)
			i--;
	}

	void prevValue() { //dn
		i++;
	}

};


#endif /* CONFIGURATORS_HPP_ */

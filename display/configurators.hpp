/*
 * configurators.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#ifndef CONFIGURATORS_HPP_
#define CONFIGURATORS_HPP_

#include "configurator.hpp"
#include "Axes.hpp"
#include "leds.hpp"
#include "radio.hpp"

class LedConf : public Configurator {
public:
	const char* opts[3] = {"Battery", "Signal Quality", "TX/RX"};

	void populate(LCDLine* lines, uint8_t nLines) {
		lines[0] << "LED Mode Select:";
		lines[1] << opts[leds.getMode()];
	}

	void nextValue() {
		uint8_t m = leds.getMode();
		m--;
		m %= 3;
		leds.setMode((Leds::Mode)m);
	}
	void prevValue() {
		uint8_t m = leds.getMode();
		m++;
		m %= 3;
		leds.setMode((Leds::Mode)m);
	}
};

class FreqConf : public Configurator {
public:
	uint8_t index;

	const char* confIndex[5] = {"Frequency", "FH Channels",
			"AFC pull-in", "TX Interval", "TX Power"};

	void init(uint8_t index) {
		this->index = index;
	}

	void populate(LCDLine* lines, uint8_t nLines) {
		lines[0].addField(confIndex[index], 12, false);
		lines[0].addField(radio.getRssi(), 4);
		switch(index) {
		case 0:
			lines[1] << radio.getFreq()/1000 << '.';
			lines[1] << radio.getFreq()%1000;
			lines[1] << "MHz";
			break;
		case 1:
			lines[1] << radio.getNumFhChannels();
			break;
		case 2:
			lines[1] << radio.getAfcPullIn();
			lines[1] << "MHz";
			break;
		case 3:
			lines[1] << radio.getTxInterval();
			lines[1] << "ms";
			break;
		case 4:
			lines[1] << (int)radio.getTxPower()*3-1;
			lines[1] << "dBm";
			break;
		}
	}

	void nextValue() {
		switch(index) {
		case 0:
			radio.setFreq(radio.getFreq()+100);
			break;
		case 1:
			radio.setNumFhChannels(radio.getNumFhChannels()+1);
			break;
		case 2: {
			float x = radio.getAfcPullIn()+0.01;
			if(x < 0.14f)
				radio.setAfcPullIn(x);
		}
			break;
		case 3:
			radio.setTxInterval(radio.getTxInterval()+1);
			break;
		case 4:
			if(radio.getTxPower() < RH_RF22_TXPOW_20DBM)
				radio.setTxPower(radio.getTxPower()+1);
			break;
		}

	}
	void prevValue() {
		switch(index) {
		case 0:
			radio.setFreq(radio.getFreq()-100);
			break;
		case 1:
			radio.setNumFhChannels(radio.getNumFhChannels()-1);
			break;
		case 2:{
			float x = radio.getAfcPullIn()-0.01f;
			if(x > 0)
			radio.setAfcPullIn(x);
		}
			break;
		case 3:
			radio.setTxInterval(radio.getTxInterval()-1);
			break;
		case 4:
			if(radio.getTxPower() > RH_RF22_TXPOW_1DBM)
				radio.setTxPower(radio.getTxPower()-1);
			break;
		}
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
			lines[0].addField(axes.getChannel(0), 5);
			lines[0].addField(axes.getChannel(1), 5);

			lines[1].addField(axes.getChannel(2), 5);
			lines[1].addField(axes.getChannel(3), 5);
			lines[1].addField(axes.getChannel(4), 5);
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

/*
 * Axes.hpp
 *
 *  Created on: Sep 4, 2014
 *      Author: walmis
 */

#ifndef AXES_HPP_
#define AXES_HPP_

#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>

using namespace xpcc;
using namespace xpcc::lpc17;

class Axes : xpcc::TickerTask {

public:
	int16_t getChannel(uint8_t ch) {
		return channels[ch];
	}

protected:
	void handleInit() {

		Pinsel::setFunc(0, 3, 2);
		Pinsel::setFunc(0, 25, 1);
		Pinsel::setFunc(0, 26, 1);
		Pinsel::setFunc(1, 30, 3);
		Pinsel::setFunc(1, 31, 3);


		ADC::enableChannel(2);
		ADC::enableChannel(3);
		ADC::enableChannel(4);
		ADC::enableChannel(5);
		ADC::enableChannel(6);

		ADC::start(ADC::ADCStartMode::START_CONTINUOUS);
	}

	void handleTick() {
		static PeriodicTimer<> t(1);

		if(t.isExpired()) {

			for(int i = 0; i < 5; i++) {
				int16_t adc = ADC::getData(i+2);

				channels[i] = (channels[i]*4-channels[i] + adc+(4/2))/4;
			}
		}
	}

	int16_t channels[5];

};


#endif /* AXES_HPP_ */

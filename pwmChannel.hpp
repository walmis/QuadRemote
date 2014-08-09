/*
 * pwmChannel.hpp
 *
 *  Created on: Apr 24, 2014
 *      Author: walmis
 */

#ifndef PWMCHANNEL_HPP_
#define PWMCHANNEL_HPP_

#include <xpcc/architecture.hpp>
#include <xpcc/math/filter/iir.hpp>

using namespace xpcc;
using namespace lpc17;

#define TMAX 210000
#define TMIN 90000

class PWMChannel : TickerTask {
public:

	PWMChannel(uint8_t port, uint8_t pin) :
			port(port), pin(pin), tUpdate(10), filter(15) {

		RitClock::initialize();

		GpioInterrupt::enableInterrupt(port, pin, IntSense::EDGE,
				IntEdge::DOUBLE);
	}

	bool isActive() {
		return active;
		//return diffFilter.getValue() < 100000;
	}

	uint16_t getDuty() {
		return (filter.getValue()-TMIN)*1024/(TMAX-TMIN);
	}
//
//	Timestamp getTHigh() {
//		return t_high;
//	}
//
//	Timestamp getTLow() {
//		return t_total - t_high;
//	}
//
//	Timestamp getTTotal() {
//		return t_total;
//	}

protected:

	Timeout<> timer;
	PeriodicTimer<> tUpdate;
	uint32_t lastGood;

	void handleTick() override {

		if(tUpdate.isExpired()) {

			if(!active && count > 20) {
				active = true;
			}

			if(active) {
				median.append(t_high.getTime());
				median.update();

				lastGood = median.getValue();
			}

			filter.append(lastGood);


		}

	}

	void handleInterrupt(int irqn) override {
		if(GpioInterrupt::checkInterrupt(irqn, port, pin, IntEvent::RISING_EDGE)) {
			//diffFilter.append(abs(tot.getTime() - t_total.getTime()));
			t_rise = RitClock::now();

		}
		if(GpioInterrupt::checkInterrupt(irqn, port, pin, IntEvent::FALLING_EDGE)) {
			if(t_rise.getTime() > 0) {
				t_high = RitClock::now() - t_rise;

				if(t_high > TMAX || t_high < TMIN) {
					active = false;
					count = 0;
				} else {
					count++;
				}

				t_rise = 0;
			}
		}
	}

	xpcc::filter::LPF<uint32_t> filter;

	xpcc::filter::Median<uint32_t, 3> median;

	volatile bool active;
	volatile int count;

	Timestamp t_high;
	Timestamp t_rise;

private:
	uint8_t port;
	uint8_t pin;
};


#endif /* PWMCHANNEL_HPP_ */

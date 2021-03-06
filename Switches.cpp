/*
 * Switches.cpp
 *
 *  Created on: Oct 10, 2014
 *      Author: walmis
 */

#include <Switches.h>
#include "pindefs.hpp"

bool Switches::getState(uint8_t sw) {
	switch(sw) {
	case 0:
		return !auxSw1::read();
	case 1:
		return !auxSw2::read();
	case 2:
		return !auxSw3::read();
	case 3:
		return !state_aux4;
	case 4:
		return !state_aux5;
	default:
		return false;
	}
}

uint8_t Switches::getBitmask() {
	uint8_t st = 0;
	for(uint8_t i= 0; i < 5; i++) {
		st |= (getState(i)&1)<<i;
	}
	return st;
}

Switches::Switches() {
}

void Switches::handleTick() {
	static xpcc::PeriodicTimer<> t(20);
	if(t.isExpired()) {
		bool s1 = auxSw4mux::read();
		bool s2 = auxSw5mux::read();

		auxSw4mux::setInput();
		auxSw5mux::setInput();

		xpcc::delay_us(5);

		state_aux4 = auxSw4mux::read();
		state_aux5 = auxSw5mux::read();

		auxSw4mux::setOutput(s1);
		auxSw5mux::setOutput(s2);

	}
}

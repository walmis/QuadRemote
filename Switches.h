/*
 * Switches.h
 *
 *  Created on: Oct 10, 2014
 *      Author: walmis
 */

#ifndef SWITCHES_H_
#define SWITCHES_H_

#include <xpcc/architecture.hpp>

class Switches : xpcc::TickerTask {
public:

	Switches();

	bool getState(uint8_t sw);

private:
	void handleTick();

	bool state_aux4 : 1;
	bool state_aux5 : 1;
};

extern Switches switches;

#endif /* SWITCHES_H_ */

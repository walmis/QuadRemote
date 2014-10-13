/*
 * mavHandler.cpp
 *
 *  Created on: Oct 13, 2014
 *      Author: walmis
 */

#include <mavHandler.hpp>
#include <remote_control.hpp>

using namespace xpcc;
using namespace xpcc::lpc17;

MAVHandler::MAVHandler() {
}

void MAVHandler::handleTick() {
	uint16_t rx_avail, tx_avail;
	if((rx_avail = btUart.rxAvailable()) && (tx_avail = radio.txAvailable())) {
		while(rx_avail && tx_avail) {
			radio.write(btUart.read());
			rx_avail--;
			tx_avail--;
		}
	}

	if(radio.rxAvailable()) {
		int16_t c;
		while(btUart.txAvailable() && (c = radio.read()) != -1) {
			btUart.write(c);
		}
	}
}

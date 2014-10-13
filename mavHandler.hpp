/*
 * mavHandler.hpp
 *
 *  Created on: Oct 13, 2014
 *      Author: walmis
 */

#ifndef MAVHANDLER_HPP_
#define MAVHANDLER_HPP_

#include <xpcc/architecture.hpp>

class MAVHandler : xpcc::TickerTask {
public:
	MAVHandler();

protected:
	void handleTick();

};

extern MAVHandler mavHandler;
#endif /* MAVHANDLER_HPP_ */

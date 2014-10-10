/*
 * Axes.hpp
 *
 *  Created on: Sep 4, 2014
 *      Author: walmis
 */

#ifndef AXES_HPP_
#define AXES_HPP_

#include <system.hpp>

using namespace xpcc;
using namespace xpcc::lpc17;

#define AXIS_YAW_CH       4
#define AXIS_ROLL_CH      2
#define AXIS_PITCH_CH     3
#define AXIS_THROTTLE_CH  1
#define AXIS_AUX6_CH      0

class Axes : xpcc::TickerTask {

public:
	int16_t getChannel(uint8_t ch);

	int16_t getRawChannel(uint8_t ch) {
		return channels[ch];
	}
	void calibration(bool en);

protected:
	void handleInit();
	void handleTick();
	int16_t channels[5];
	uint16_t calData[5][2];
	bool calibrating;
};

extern Axes axes;

#endif /* AXES_HPP_ */

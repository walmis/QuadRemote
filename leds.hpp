/*
 * Leds.h
 *
 *  Created on: Sep 24, 2014
 *      Author: walmis
 */

#ifndef LEDS_H_
#define LEDS_H_

class Leds final : TickerTask {
public:
	enum Mode {
		MODE_BATTERY = 0,
		MODE_SIGNAL_Q,
		MODE_TXRX,
		MODE_REMOTE_BATTERY,

		MODE_LAST
	};

	Mode getMode();
	void setMode(Mode mode);

private:
	void handleInit();
	void handleTick();

	Mode mode;
};

extern Leds leds;

#endif /* LEDS_H_ */

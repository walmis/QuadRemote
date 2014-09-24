/*
 * Leds.h
 *
 *  Created on: Sep 24, 2014
 *      Author: walmis
 */

#ifndef LEDS_H_
#define LEDS_H_

class Leds : TickerTask {
public:
	enum Mode {
		MODE_BATTERY = 0,
		MODE_SIGNAL_Q,
		MODE_TXRX
	};

	Mode getMode();
	void setMode(Mode mode);

private:
	void handleInit();
	void handleTick();

	Mode mode;
};

#endif /* LEDS_H_ */

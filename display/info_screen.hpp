/*
 * info_screen.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#ifndef INFO_SCREEN_HPP_
#define INFO_SCREEN_HPP_

#include "configurator.hpp"
#include "lcd_display.hpp"

class InfoScreen {
public:
	InfoScreen() : prev(0), next(0) {}

	virtual void populate(LCDLine* lines, uint8_t nLines) = 0;

	virtual Configurator* getConfigurator(uint8_t i) {
		return 0;
	}

	InfoScreen* prev;
	InfoScreen* next;
};


#endif /* INFO_SCREEN_HPP_ */

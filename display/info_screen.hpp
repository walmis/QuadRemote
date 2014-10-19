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
	static bool populate(LCDLine* lines, uint8_t nLines) {
		return false;
	};
	static Configurator* getConfigurator(uint8_t i) {
		return 0;
	}
};


#endif /* INFO_SCREEN_HPP_ */

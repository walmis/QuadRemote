/*
 * display.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#ifndef DISPLAY_HPP_
#define DISPLAY_HPP_

#include <xpcc/ui/button_group.hpp>
#include "lcd_display.hpp"
#include <system.hpp>

//#include "info_screen.hpp"
//#include "configurator.hpp"
class InfoScreen;
class Configurator;

class Display : public LcdDisplay<Hd44780Lcd> {
public:
	Display(Hd44780Lcd& lcd) : LcdDisplay<Hd44780Lcd>(&lcd), buttons(0xF, 50, 8) {
		conf = 0;
		currentScreen = 0;
		confItem = 0;
		screens_tail = 0;
	}

	void handleTick() override;

	void updateButtons();

	void getPage(uint8_t page);

	void addScreen(InfoScreen* screen);

	uint8_t confItem;

	//linked list tail
	InfoScreen* screens_tail;
	//currently displayed screen
	InfoScreen* currentScreen;

	Configurator* conf;

	xpcc::ButtonGroup<> buttons;
};



#endif /* DISPLAY_HPP_ */

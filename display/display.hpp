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
#include "configurator.hpp"
#include "info_screen.hpp"


template <class... Params>
void f(Params... params) {
    std::array<int, sizeof...(params)> list = {params...};
}

template<typename... Screens>
class Display : public LcdDisplay<Hd44780Lcd> {
public:
	Display(Hd44780Lcd& lcd) : LcdDisplay<Hd44780Lcd>(&lcd), buttons(0xF, 50, 8) {
		conf = 0;
		currentScreen = 0;
		confItem = 0;
	}

	void handleTick() override;

	void updateButtons();

	void getPage(uint8_t page);

	uint8_t confItem;

	//currently displayed screen
	uint8_t currentScreen;

	Configurator* conf;

	xpcc::ButtonGroup<> buttons;

	typedef bool (*PopulateFunc)(LCDLine* lines, uint8_t nLines);
	typedef Configurator* (*getConfiguratorFunc)(uint8_t i);

	static const unsigned short int n_screens = sizeof...(Screens);
	const PopulateFunc populate[n_screens] = {&Screens::populate...};
	const getConfiguratorFunc getConfigurator[n_screens] = {&Screens::getConfigurator...};
};

template<typename... Screens>
void Display<Screens...>::handleTick()
{
	LcdDisplay<Hd44780Lcd>::handleTick();
	static xpcc::PeriodicTimer<> t(10);
	if (t.isExpired()) {
		updateButtons();
	}
}

template<typename... Screens>
void Display<Screens...>::updateButtons() {
	buttons.update(joystickBtns::read());
	if (buttons.isPressed(BUTTON_UP) || buttons.isRepeated(BUTTON_UP)) {
		if (conf) {
			conf->nextValue();
		} else {
			if (currentScreen > 0) {
				currentScreen--;
			} else {
				currentScreen = n_screens-1;
			}
		}
	}
	if (buttons.isPressed(BUTTON_DN) || buttons.isRepeated(BUTTON_DN)) {
		if (conf) {
			conf->prevValue();
		} else {
			if (currentScreen < n_screens-1) {
				currentScreen++;
			} else {
				currentScreen = 0;
			}
		}
	}
	if (buttons.isPressed(BUTTON_RIGHT)) {
		if (n_screens) {

			if (!conf) {
				confItem = 0;
				conf = getConfigurator[currentScreen](confItem);
				if(conf)
					conf->init(confItem);
			} else {
				confItem++;
				conf = getConfigurator[currentScreen](confItem);
				if(conf)
					conf->init(confItem);
			}
		}
	}
	if (buttons.isPressed(BUTTON_LEFT)) {
		if (n_screens) {
			if (conf) {
				if (confItem == 0) {
					conf = 0;
				} else {
					confItem--;
					conf = getConfigurator[currentScreen](confItem);
					if(conf)
						conf->init(confItem);
				}
			}
		}
	}
}

template<typename... Screens>
void Display<Screens...>::getPage(uint8_t page) {
	if (conf) {
		conf->populate(line, 2);
	} else if (n_screens) {
		populate[currentScreen](line, 2);
	}
}

#endif /* DISPLAY_HPP_ */

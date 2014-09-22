/*
 * display.cpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#include "display.hpp"
#include "configurator.hpp"
#include "info_screen.hpp"

void Display::handleTick()
{
	LcdDisplay<Hd44780Lcd>::handleTick();
	static PeriodicTimer<> t(10);
	if (t.isExpired()) {
		updateButtons();
	}
}

void Display::updateButtons() {
	buttons.update(joystickBtns::read());
	if (buttons.isPressed(BUTTON_UP) || buttons.isRepeated(BUTTON_UP)) {
		if (conf) {
			conf->nextValue();
		} else {
			if (currentScreen && currentScreen->prev) {
				currentScreen = currentScreen->prev;
			}
		}
	}
	if (buttons.isPressed(BUTTON_DN) || buttons.isRepeated(BUTTON_DN)) {
		if (conf) {
			conf->prevValue();
		} else {
			if (currentScreen && currentScreen->next) {
				currentScreen = currentScreen->next;
			} else {
				currentScreen = screens_tail;
			}
		}
	}
	if (buttons.isPressed(BUTTON_RIGHT)) {
		if (currentScreen) {

			if (!conf) {
				confItem = 0;
				conf = currentScreen->getConfigurator(confItem);
				if(conf)
					conf->init();
			} else {
				confItem++;
				conf = currentScreen->getConfigurator(confItem);
			}
		}
	}
	if (buttons.isPressed(BUTTON_LEFT)) {
		if (currentScreen) {
			if (conf) {
				if (confItem == 0) {
					conf = 0;
				} else {
					confItem--;
					conf = currentScreen->getConfigurator(confItem);
				}
			}
		}
	}
}

void Display::getPage(uint8_t page) {
	if (conf) {
		conf->populate(line, 2);
	} else if (currentScreen) {
		currentScreen->populate(line, 2);
	}
}

void Display::addScreen(InfoScreen* screen) {
	if (!screens_tail) {
		screens_tail = screen;
		currentScreen = screen;
	} else {
		InfoScreen* s = screens_tail;
		while (s->next) {
			s = s->next;
		}
		s->next = screen;
		screen->prev = s;
	}
}



/*
 * pindefs.hpp
 *
 *  Created on: Mar 15, 2013
 *      Author: walmis
 */

#ifndef PINDEFS_HPP_
#define PINDEFS_HPP_

#include <xpcc/architecture.hpp>

#define AD_CH_VBAT 7

//GPIO__INPUT(progPin, 2, 0);

GPIO__OUTPUT(_ledGreen, 0, 15);
GPIO__OUTPUT(_ledYellow, 0, 17);
GPIO__OUTPUT(_ledRed, 0, 22);

typedef xpcc::gpio::Invert<_ledRed> ledRed;
typedef xpcc::gpio::Invert<_ledGreen> ledGreen;
typedef xpcc::gpio::Invert<_ledYellow> ledYellow;

GPIO__OUTPUT(usbConnPin, 1, 18);

GPIO__OUTPUT(lcd_e, 1, 4);
GPIO__OUTPUT(lcd_rs, 1, 1);

GPIO__IO(lcd_d4, 1, 14);
GPIO__IO(lcd_d5, 1, 10);
GPIO__IO(lcd_d6, 1, 9);
GPIO__IO(lcd_d7, 1, 8);

GPIO__INPUT(btnL, 2, 4);
GPIO__INPUT(btnU, 2, 5);
GPIO__INPUT(btnR, 2, 6);
GPIO__INPUT(btnD, 2, 7);

#define BUTTON_LEFT (1<<3)
#define BUTTON_RIGHT (1<<2)
#define BUTTON_UP (1<<1)
#define BUTTON_DN (1<<0)

typedef xpcc::gpio::Nibble<btnL, btnR, btnU, btnD> joystickBtns;

GPIO__INPUT(auxSw5_, 2, 8);
typedef xpcc::gpio::Invert<auxSw5_> auxSw5;

typedef xpcc::gpio::Nibble<lcd_d7, lcd_d6, lcd_d5, lcd_d4> lcd_data;

GPIO__IO(radio_sel, 2, 1);
GPIO__IO(radio_irq, 2, 0);

typedef xpcc::lpc17::SpiMaster1 radioSpiMaster;

#endif /* PINDEFS_HPP_ */

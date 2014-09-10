/*
 * pindefs.hpp
 *
 *  Created on: Mar 15, 2013
 *      Author: walmis
 */

#ifndef PINDEFS_HPP_
#define PINDEFS_HPP_

#include <xpcc/architecture.hpp>


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

typedef xpcc::gpio::Nibble<lcd_d7, lcd_d6, lcd_d5, lcd_d4> lcd_data;

GPIO__IO(radio_sel, 2, 1);
GPIO__IO(radio_irq, 2, 0);

#endif /* PINDEFS_HPP_ */

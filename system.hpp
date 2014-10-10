#ifndef _SYSTEM_HPP_
#define _SYSTEM_HPP_

extern void panic(const char* str);

#include "pindefs.hpp"
#include "eeprom/eeprom.hpp"
#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/driver/display/hd44780.hpp>

typedef xpcc::Hd44780<lcd_e, xpcc::gpio::Unused, lcd_rs, lcd_data> Hd44780Lcd;

extern Hd44780Lcd lcd;



#endif

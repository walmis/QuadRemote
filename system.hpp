#ifndef _SYSTEM_HPP_
#define _SYSTEM_HPP_

extern void panic(const char* str);

#include "pindefs.hpp"
#include "eeprom/eeprom.hpp"
#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/driver/display/hd44780.hpp>

typedef xpcc::Hd44780<lcd_e, xpcc::gpio::Unused, lcd_rs, lcd_data> Hd44780Lcd;

#include "display/display.hpp"

#include "battery.hpp"
#include "Axes.hpp"

#include "remote_control.hpp"
#include "display/lcd_display.hpp"

//forward declarations
class Display;
class Axes;
class RemoteControl;

extern Display disp;
extern Battery battery;
extern Axes axes;
extern Hd44780Lcd lcd;
extern RemoteControl radio;



#endif

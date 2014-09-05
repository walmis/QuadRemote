/*
 * main.cpp
 *
 *  Created on: Feb 28, 2013
 *      Author: walmis
 */

#define TRP_DEBUG

#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>

#include "pindefs.hpp"

#include <xpcc/driver/connectivity/wireless/at86rf230.hpp>
#include <xpcc/driver/connectivity/usb/USBDevice.hpp>
#include <xpcc/driver/storage/i2c_eeprom.hpp>

#include <xpcc/communication/TinyRadioProtocol.hpp>

#include <lpc17xx_nvic.h>
#include <lpc17xx_uart.h>
#include <lpc17xx_pinsel.h>

#include <xpcc/debug.hpp>
#include <xpcc/math/filter.hpp>

#include <xpcc/io/terminal.hpp>

#include <math.h>

#include <xpcc/driver/display.hpp>

#include "Axes.hpp"

#include <RH_RF22.h>

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "QuadRmV0.1";

RH_RF22 radio(0, 1);

class UARTDevice : public IODevice {

public:
	UARTDevice(int baud) {

		UART_CFG_Type cfg;
		UART_FIFO_CFG_Type fifo_cfg;

		UART_ConfigStructInit(&cfg);
		cfg.Baud_rate = baud;

		UART_Init(LPC_UART0, &cfg);

		PINSEL_CFG_Type PinCfg;

		PinCfg.Funcnum = 1;
		PinCfg.OpenDrain = 0;
		PinCfg.Pinmode = 0;
		PinCfg.Pinnum = 2;
		PinCfg.Portnum = 0;
		PINSEL_ConfigPin(&PinCfg);
		PinCfg.Pinnum = 3;
		PINSEL_ConfigPin(&PinCfg);

		UART_Init(LPC_UART0, &cfg);

		UART_FIFOConfigStructInit(&fifo_cfg);

		UART_FIFOConfig(LPC_UART0, &fifo_cfg);

		UART_TxCmd(LPC_UART0, ENABLE);
	}

	void
	write(char c) {
		while (!(LPC_UART0->LSR & UART_LSR_THRE)) {
		}

		UART_SendByte(LPC_UART0, c);
	};

	void
	flush(){};

	/// Read a single character
	bool read(char& c) {
		if((LPC_UART0->LSR & 1)) {
			c = (LPC_UART0->RBR & UART_RBR_MASKBIT);
			return true;
		}
		return false;
	}
};

#define _DEBUG
//#define _SER_DEBUG

UARTDevice uart(460800);

USBSerial device(0xffff);
xpcc::IOStream stream(device);
xpcc::NullIODevice null;

#ifdef _DEBUG
xpcc::log::Logger xpcc::log::info(device);
xpcc::log::Logger xpcc::log::debug(device);
xpcc::log::Logger xpcc::log::error(device);
xpcc::log::Logger xpcc::log::warning(device);
#else
#ifdef _SER_DEBUG
xpcc::log::Logger xpcc::log::info(uart);
xpcc::log::Logger xpcc::log::debug(uart);
xpcc::log::Logger xpcc::log::error(uart);
xpcc::log::Logger xpcc::log::warning(uart);
#else
xpcc::log::Logger xpcc::log::info(null);
xpcc::log::Logger xpcc::log::debug(null);
xpcc::log::Logger xpcc::log::error(null);
#endif
#endif


class CmdTerminal : public Terminal {
public:
	CmdTerminal(IODevice& device) : Terminal(device), ios(device) {

	};

protected:
	xpcc::IOStream ios;

	void handleCommand(uint8_t nargs, char* argv[]) {

		if(cmp(argv[0], "reset")) {
			NVIC_SystemReset();
		}

		else if(cmp(argv[0], "flash")) {

			LPC_WDT->WDFEED = 0x56;
		}

		else if(cmp(argv[0], "version")) {

			ios << fwversion << endl;
		}
		else if(cmp(argv[0], "radio")) {
			printf("init radio\n");
			if(!radio.init()) {
				XPCC_LOG_DEBUG. printf("radio init failed\n");
			}

		}

	}

};

CmdTerminal cmd(device);
CmdTerminal ucmd(uart);


void idle() {
	static PeriodicTimer<> t(500);

	if(t.isExpired()) {

		LPC_WDT->WDFEED = 0xAA;
		LPC_WDT->WDFEED = 0x55;
	}
}

Hd44780<lcd_e, gpio::Unused, lcd_rs, lcd_data> lcd(16, 2);
Axes axes;


class LCDLine {
public:
	LCDLine() {
		clear();
	}

	char line[16];
	uint8_t pos;

	void clear() {
		memset(line, ' ', sizeof(line));
		pos = 0;
	}

	void addTextField(char *text, uint8_t fieldWidth) {
		StringStream<16> str;

		str << text;

		addField(str, fieldWidth);
	}

	void addIntegerField(int i, uint8_t fieldWidth) {
		StringStream<16> str;

		str << i;

		addField(str, fieldWidth);
	}

	void addFloatField(float n) {


	}

private:

	void addField(StringStream<16> &str, uint8_t fieldWidth) {
		alignField(str, fieldWidth);
		strncpy(line+pos, str.buffer, fieldWidth);
		pos += fieldWidth;
	}

	void alignField(StringStream<16> &str, uint8_t fieldWidth) {
		if(str.pos < fieldWidth) {
			uint8_t diff = fieldWidth - str.pos;

			memmove(str.buffer+diff, str.buffer, str.pos);
			memset(str.buffer, ' ', diff);
		}
	}


};

class LcdDisplay : TickerTask {

	void handleTick() {
		static int i = 0;

		i++;

		LCDLine line;

		line.addIntegerField(axes.getChannel(0), 5);
		line.addIntegerField(axes.getChannel(1), 5);
		line.addIntegerField(axes.getChannel(2), 5);

		lcd.setCursor(0, 0);
		lcd.IOStream::write((uint8_t*)line.line, 16);

		line.clear();
		line.addIntegerField(axes.getChannel(3), 5);
		line.addIntegerField(axes.getChannel(4), 5);


		lcd.IOStream::write((uint8_t*)line.line, 16);
	}
};

LcdDisplay test;





int main() {
	lpc17::RitClock::initialize();
	lpc17::SysTickTimer::enable();

	ADC::init(50000);
	ADC::burstMode(true);

	//lpc17::SysTickTimer::attachInterrupt(sysTick);

	//Pinsel::setFunc(0, 7, 2); //SCK0
	//Pinsel::setFunc(0, 8, 2); //MISO0
	//Pinsel::setFunc(0, 9, 2); //MOSI0

	//SpiMaster1::initialize(SpiMaster1::Mode::MODE_0, 8000000);

	xpcc::Random::seed();

	Pinsel::setFunc(0, 10, 2);
	Pinsel::setFunc(0, 11, 2);

	//lpc17::I2cMaster2::initialize<xpcc::I2cMaster::DataRate::Fast>();

	usbConnPin::setOutput(true);
	device.connect();

	NVIC_SetPriority(USB_IRQn, 10);
	NVIC_SetPriority(EINT3_IRQn, 0);

	delay_ms(50);
	lcd.initialize();

	ledRed::setOutput(1);

	TickerTask::tasksRun(idle);
}

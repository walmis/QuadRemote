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
#include "Switches.h"

#include <xpcc/driver/connectivity/usb/USBDevice.hpp>
#include <xpcc/architecture/peripheral/i2c_adapter.hpp>
#include "eeprom/eeprom.hpp"
#include "radio.hpp"
#include "remote_control.hpp"
#include "Axes.hpp"
#include "leds.hpp"
#include "display/display.hpp"
#include "battery.hpp"

#include <xpcc/debug.hpp>
#include <xpcc/math/filter.hpp>
#include <xpcc/io/terminal.hpp>

#include <math.h>
#include "system.hpp"
#include "display/screens.hpp"

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "QuadRmV0.4";

RemoteControl radio;
Hd44780Lcd lcd(16, 2);
Axes axes;
Battery battery;
Display disp(lcd);
Leds leds;
Switches switches;

#define _DEBUG
//#define _SER_DEBUG

USBSerial device(0xffff, 0xce38);
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

void idle() {
	static PeriodicTimer<> t(500);

	if(t.isExpired()) {

		LPC_WDT->WDFEED = 0xAA;
		LPC_WDT->WDFEED = 0x55;
	}
}


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
		else if (cmp(argv[0], "scan")) {
			xpcc::I2cWriteAdapter adapter;

			uint8_t buf[3] = { 0x0F };

			XPCC_LOG_DEBUG << "Scanning i2c bus\n";
			for (int i = 0; i < 128; i++) {
				adapter.initialize(i, buf, 1);
				I2cMaster1::startBlocking(&adapter);

				if (I2cMaster1::getErrorState()
						!= xpcc::I2cMaster::Error::AddressNack) {
					XPCC_LOG_DEBUG.printf("Found device @ 0x%x\n", i);
				}
			}
		}
		else if(cmp(argv[0], "flash")) {

			LPC_WDT->WDFEED = 0x56;
		}
		else if(cmp(argv[0], "eeread")) {
			if(nargs == 1) {
				uint8_t buf[256];
				for(int i=0; i < 255;i++) {
					eeprom.readByte((uint16_t)i, buf[i]);
				}

				XPCC_LOG_DEBUG.dump_buffer(buf, 255);

			} else {
				uint16_t addr = toInt(argv[1]);

				uint8_t c = 0;
				if(!eeprom.readByte(addr, c)) {
					printf("Failed\n");
				} else {
					printf("> %02x\n", c);
				}
			}
		}
		else if(cmp(argv[0], "eewrite")) {
			if(nargs == 1) {
				uint8_t buf[256];
				for(int i = 0; i < 255; i++) {
					buf[i] = i;
				}

				eeprom.write(0, buf, 255);

			} else {
				uint16_t addr = toInt(argv[1]);
				uint16_t b = toInt(argv[2]);

				uint8_t c = 0;
				if(!eeprom.writeByte(addr, b)) {
					printf("Failed\n");
				} else {
					printf("OK\n");
				}
			}
		}
		else if(cmp(argv[0], "flashboot")) {
			uint16_t binLen;
			uint16_t len = binLen = toInt(argv[1]);
			uint32_t checksum = toInt(argv[2]);

			if(len < 3000 || len > 4095) {
				printf("ERR: bad length\n");
				return;
			} else {
				printf("OK\n");
			}

			uint32_t flashPos = 0x00000000;

			IAP iap;

			uint16_t pos = 0;
			uint8_t buf[256];
			memset(buf, 0xFF, 256);

			lcd.clear();
			lcd << "Flash boot\n";

			while(len) {

				while(len && pos < sizeof(buf)) {
					int16_t c;
					do c = device.read(); while(c<0);
					buf[pos] = c;
					pos++;
					len--;
				}

				if(pos) {
					iap.findErasePrepareSector(flashPos);
					iap.writeData(flashPos, (unsigned int*)buf, sizeof(buf));
					printf("write %x %d\n", flashPos, pos);
					flashPos += sizeof(buf);
					pos = 0;
					memset(buf, 0xFF, 256);
				}
			}
			uint32_t sum = 0;
			uint8_t* ptr = (uint8_t*)0x00000000;

			for(int i = 0; i < binLen; i++) {
				sum += *ptr++;
			}
			printf("checksum: %08x %08x\n", sum, checksum);

		}
		else if(cmp(argv[0], "dumpboot")) {
			XPCC_LOG_DEBUG.dump_buffer((uint8_t*)0x00000000, 4096);
		}
		else if(cmp(argv[0], "bootsum")) {
			uint32_t sum = 0;
			uint8_t* ptr = (uint8_t*)0x00000000;

			for(int i = 0; i < 4096; i++) {
				sum += *ptr++;
			}
			printf("checksum: %08x\n", sum);
		}

		else if(cmp(argv[0], "version")) {

			ios << fwversion << endl;
		}

		else if(cmp(argv[0], "rssi")) {

			printf("rssi %d\n", radio.rssiRead() / 2 - 120 );
		}

		else if(cmp(argv[0], "freq")) {
			int f = toInt(argv[1]);

			radio.setFrequency((float)f, 0.05);
		}

		else if(cmp(argv[0], "page")) {
			int page = toInt(argv[1]);

			disp.setPage(page);

		}

		else if(cmp(argv[0], "dumpadc")) {
			XPCC_LOG_DEBUG .printf("PCADC %d\n", CLKPwr::getPCLK(CLKPwr::ClkType::ADC));
			XPCC_LOG_DEBUG .printf("ADCR 0x%x\n", LPC_ADC->ADCR);

			uint8_t ch = 0;
			for(int ch = 0; ch < 8; ch++) {
				XPCC_LOG_DEBUG .printf("[%d] %d\n", ch, ADC::getData(ch));
			}


		}
	}
};


CmdTerminal cmd(device);
//CmdTerminal ucmd(uart);

BatScreen screen_bat;
AxesScreen screen_ax;
RadioScreen screen_radio;

InfoScreen* const screens[] = {
		&screen_bat,
		&screen_ax,
		&screen_radio
};

void panic(const char* str) {
	lcd.clear();
	lcd << "PANIC:" << str;

	PeriodicTimer<> t(5000);
	while(!t.isExpired());

	NVIC_SystemReset();
}

int main() {
	RitClock::initialize();
	SysTickTimer::enable();
	delay_ms(1);

	ADC::init();
	ADC::enableChannel(AD_CH_VBAT); //VBAT
	Pinsel::setFunc(0, 2, 2); //AD0[7]
	ADC::start(ADC::ADCStartMode::START_CONTINUOUS);

	Uart2::init(460800);

	Pinsel::setFunc(0, 10, 1); //TXD2
	Pinsel::setFunc(0, 11, 1); //RXD2

	//lpc17::SysTickTimer::attachInterrupt(sysTick);

	///** Init SPI1 **///
	SpiMaster1::initialize(SpiMaster1::Mode::MODE_0, 8000000);
	Pinsel::setFunc(0, 7, 2); //SCK1
	Pinsel::setFunc(0, 8, 2); //MISO1
	Pinsel::setFunc(0, 9, 2); //MOSI1
	//////

	xpcc::Random::seed();

	////////
	lpc17::I2cMaster1::initialize<xpcc::I2cMaster::DataRate::Fast>();
	Pinsel::setFunc(0, 0, 3); //SDA1
	Pinsel::setFunc(0, 1, 3); //SCL1
	////////

	usbConnPin::setOutput(true);
	device.connect();

	NVIC_SetPriority(USB_IRQn, 10);
	NVIC_SetPriority(EINT3_IRQn, 0);

	delay_ms(50);

	lcd.initialize();

	GpioInt::enableInterrupts();

	ledRed::setOutput(0);
	ledYellow::setOutput(0);
	ledGreen::setOutput(0);

	eeprom.initialize();

	for(auto s : screens) {
		disp.addScreen(s);
	}

	TickerTask::tasksRun(idle);
}

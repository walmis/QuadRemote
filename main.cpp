/*
 * main.cpp
 *
 *  Created on: Feb 28, 2013
 *      Author: walmis
 */

#define TRP_DEBUG

#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>
#include <xpcc/ui/button_group.hpp>

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

#include "radio.hpp"

#include "LcdDisplay.hpp"

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "QuadRmV0.1";




class RemoteControl : public Radio {
public:
	struct RCPacket {
		int16_t yawCh;
		int16_t pitchCh;
		int16_t rollCh;
		uint8_t throttleCh;
		uint8_t auxCh;
		uint8_t switches;
		uint8_t reserved[4];
	} __attribute__((packed));

	RCPacket rcData;

	uint8_t noiseFloor;

	ProfileTimer p;

protected:
	void handleTick() {

		static PeriodicTimer<> t(10);

		if(t.isExpired() && !transmitting()) {
			noiseFloor = rssiRead();

			p.start();
			send((uint8_t*)(&rcData), sizeof(rcData));
			sending = true;
		}

		if(sending && !transmitting()) {
			sending = false;
			p.end();
			setModeRx();


		}
	}
	bool sending;


};

RemoteControl radio;

#define _DEBUG
//#define _SER_DEBUG

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

void idle() {
	static PeriodicTimer<> t(500);

	if(t.isExpired()) {

		LPC_WDT->WDFEED = 0xAA;
		LPC_WDT->WDFEED = 0x55;
	}
}

typedef Hd44780<lcd_e, gpio::Unused, lcd_rs, lcd_data> Hd44780Lcd;

Hd44780Lcd lcd(16, 2);
Axes axes;

class Battery {
public:
	/* voltages (millivolt) of 0%, 10%, ... 100% */
	const unsigned short percent_to_volt_discharge[2][11] = {
	/* measured values */
	{ 2800 / 3, 3250 / 3, 3410 / 3, 3530 / 3, 3640 / 3, 3740 / 3, 3850 / 3, 3950
			/ 3, 4090 / 3, 4270 / 3, 4750 / 3 }, /* Alkaline */
	{ 3100 / 3, 3550 / 3, 3630 / 3, 3690 / 3, 3720 / 3, 3740 / 3, 3760 / 3, 3780
			/ 3, 3800 / 3, 3860 / 3, 4050 / 3 } /* NiMH */
	};

	void update() {

		int16_t adc = ADC::getData(AD_CH_VBAT);

		//4096 - 3.3V
		//30k 8.2k

		float bat = (adc / (4096 / 3.3f)) * 4.651f;

		if(packVoltage < 0.1) {
			packVoltage = bat;
		} else {
			packVoltage = (packVoltage*50 + bat) / 51;
		}

		cellVoltage =  packVoltage / 8;

		//XPCC_LOG_DEBUG << adc << " "<< packVoltage << endl;
	}

	int getBatteryPercent() {
		return voltage_to_percent(cellVoltage*1000,
				percent_to_volt_discharge[1]);
	}

	bool onUSBPower() {
		if(packVoltage < 4.5 && packVoltage > 4.1) {
			return true;
		}
		return false;
	}

	Fp32f<16> getPackVoltage(){
		return packVoltage;
	}

	Fp32f<16> getCellVoltage(){
		return cellVoltage;
	}

private:
	float cellVoltage;
	float packVoltage;

	/* look into the percent_to_volt_* table and get a realistic battery level */
	int voltage_to_percent(int voltage, const unsigned short* table) {
		if (voltage <= table[0]) {
			return 0;
		} else if (voltage >= table[10]) {
			return 100;
		} else {
			/* search nearest value */
			int i = 0;
			while (i < 10 && table[i + 1] < voltage)
				i++;
			/* interpolate linear between the smaller and greater value */
			/* Tens digit, 10% per entry, ones digit: interpolated */
			return i * 10
					+ (voltage - table[i]) * 10 / (table[i + 1] - table[i]);
		}
	}
};

Battery battery;

class MyDisplay : public LcdDisplay<Hd44780Lcd> {
public:
	MyDisplay() : LcdDisplay<Hd44780Lcd>(&lcd), buttons(0xF) {

	}

	void handleTick() override {
		LcdDisplay<Hd44780Lcd>::handleTick();

		static PeriodicTimer<> t(10);
		if(t.isExpired()) {
			updateButtons();
		}
	}

	void updateButtons() {
		buttons.update(joystickBtns::read());

		if(buttons.isPressed(BUTTON_UP)) {
			page--;
		}

		if(buttons.isPressed(BUTTON_DN)) {
			page++;
		}
	}

	void getPage(uint8_t page) {
		switch(page) {
		default:
			setPage(0);

		case 0:
			battery.update();

			line[0].addTextField("SYS");

			if(!battery.onUSBPower()) {
				line[1].addField(battery.getBatteryPercent(), 3);
				line[1].addField("% ");

				line[1].addField(battery.getPackVoltage(), 4);
				line[1].addField('V');
			} else {
				line[1].addField("VUSB");
			}

			break;
		case 1:
			line[0].addIntegerField(axes.getChannel(0), 5);
			line[0].addIntegerField(axes.getChannel(1), 5);
			line[0].addIntegerField(axes.getChannel(2), 5);

			line[1].addIntegerField(axes.getChannel(3), 5);
			line[1].addIntegerField(axes.getChannel(4), 5);

			line[1].addIntegerField(btnL::read(), 2);
			line[1].addIntegerField(btnR::read(), 1);
			line[1].addIntegerField(btnU::read(), 1);
			line[1].addIntegerField(btnD::read(), 1);
			line[1].addIntegerField(auxSw5::read(), 1);
			break;
		case 2:
		{
			int rssi = radio.lastRssi();
			int noise = (radio.noiseFloor *100 / 190) - 127;
			//int noise = ((int)radio.rssiRead()*100 / 190) - 127;

			int rxe = radio.getRxBad();
			static int txe = 0;
			txe++;
			txe %= 100;
			int tx = radio.getTxGood() % 1000;
			int rx = radio.getRxGood() % 1000;

			line[0].addTextField("T:");
			line[0].addIntegerField(tx, 3);
			line[0].addTextField("/");
			line[0].addIntegerField(txe, 2);

			line[0].addTextField(" dBm");
			line[0].addIntegerField(rssi, 4);

			line[1].addTextField("R:");
			line[1].addIntegerField(rx, 3);
			line[1].addTextField("/");
			line[1].addIntegerField(rxe, 2);

			line[1].addIntegerField(noise, 8);
			break;
		}

		}
	}

	xpcc::ButtonGroup<> buttons;
};

MyDisplay disp;

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

		else if(cmp(argv[0], "rssi")) {

			printf("rssi %d\n", radio.rssiRead() / 2 - 120 );
		}

		else if(cmp(argv[0], "send")) {
			{
				PROFILE();
				radio.sendTest();
			}
		}
		else if(cmp(argv[0], "freq")) {
			int f = to_int(argv[1]);

			radio.setFrequency((float)f, 0.05);
		}



		else if(cmp(argv[0], "page")) {
			int page = to_int(argv[1]);

			disp.setPage(page);

		}
	}
};



CmdTerminal cmd(device);
//CmdTerminal ucmd(uart);


int main() {
	lpc17::RitClock::initialize();
	lpc17::SysTickTimer::enable();

	ADC::init(50000);
	ADC::burstMode(true);

	ADC::enableChannel(AD_CH_VBAT); //VBAT
	Pinsel::setFunc(0, 2, 2); //AD0[7]


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

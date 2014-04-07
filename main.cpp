/*
 * main.cpp
 *
 *  Created on: Feb 28, 2013
 *      Author: walmis
 */
#include <lpc17xx_nvic.h>
#include <lpc17xx_uart.h>
#include <lpc17xx_pinsel.h>

#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>

#include <xpcc/architecture/peripheral/i2c_adapter.hpp>

#include <xpcc/debug.hpp>
#include <xpcc/math/filter.hpp>

#include <xpcc/driver/connectivity/usb/USBDevice.hpp>
#include <xpcc/io/terminal.hpp>

#include <xpcc/architecture/platform/cortex_m3/lpc/lpc17/i2c_master.hpp>

#include <math.h>
#include <new>

#include "MMA8451Q.hpp"
#include "L3GD20.hpp"

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "QuadV0.01";

#include "pindefs.hpp"


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

//#define _DEBUG
#define _SER_DEBUG

UARTDevice uart(460800);

USBSerial device(0xffff);
xpcc::IOStream stream(device);

#ifdef _DEBUG
xpcc::log::Logger xpcc::log::info(device);
xpcc::log::Logger xpcc::log::debug(device);
#else
#ifdef _SER_DEBUG
xpcc::log::Logger xpcc::log::info(uart);
xpcc::log::Logger xpcc::log::debug(uart);
#else
xpcc::log::Logger xpcc::log::info(null);
xpcc::log::Logger xpcc::log::debug(null);
#endif
#endif


MMA8451Q<I2cMaster2> accel;
L3GD20<I2cMaster2> gyro;


void boot_jump( uint32_t address ){
   __asm("LDR SP, [R0]\n"
   "LDR PC, [R0, #4]");
}


xpcc::NullIODevice null;

xpcc::log::Logger xpcc::log::error(null);

enum { r0, r1, r2, r3, r12, lr, pc, psr};

extern "C" void HardFault_Handler(void)
{
  asm volatile("MRS r0, MSP;"
		       "B Hard_Fault_Handler");
}
extern "C" void UsageFault_Handler(void)
{
  asm volatile("MRS r0, MSP;"
		       "B Hard_Fault_Handler");
}
extern "C" void BusFault_Handler(void)
{
  asm volatile("MRS r0, MSP;"
		       "B Hard_Fault_Handler");
}

extern "C"
void Hard_Fault_Handler(uint32_t stack[]) {

	//register uint32_t* stack = (uint32_t*)__get_MSP();

	XPCC_LOG_DEBUG .printf("Hard Fault\n");

	XPCC_LOG_DEBUG .printf("r0  = 0x%08x\n", stack[r0]);
	XPCC_LOG_DEBUG .printf("r1  = 0x%08x\n", stack[r1]);
	XPCC_LOG_DEBUG .printf("r2  = 0x%08x\n", stack[r2]);
	XPCC_LOG_DEBUG .printf("r3  = 0x%08x\n", stack[r3]);
	XPCC_LOG_DEBUG .printf("r12 = 0x%08x\n", stack[r12]);
	XPCC_LOG_DEBUG .printf("lr  = 0x%08x\n", stack[lr]);
	XPCC_LOG_DEBUG .printf("pc  = 0x%08x\n", stack[pc]);
	XPCC_LOG_DEBUG .printf("psr = 0x%08x\n", stack[psr]);


	while(1) {
		if(!progPin::read()) {
			for(int i = 0; i < 10000; i++) {}
			NVIC_SystemReset();
		}
	}
}

void sysTick() {

//	LPC_WDT->WDFEED = 0xAA;
//	LPC_WDT->WDFEED = 0x55;

//	if(progPin::read() == 0) {
//		//NVIC_SystemReset();
//
//		NVIC_DeInit();
//
//		usbConnPin::setOutput(false);
//		delay_ms(100);
//
//		LPC_WDT->WDFEED = 0x56;
//	}

}

class CmdTerminal : public Terminal {
public:
	CmdTerminal(IODevice& device) : Terminal(device) {

	};

protected:

	void handleCommand(uint8_t nargs, char* argv[]) {

		if(cmp(argv[0], "start")) {

			XPCC_LOG_DEBUG .printf("starting\n");

			static xpcc::I2cWriteAdapter adapter;

			static uint8_t buf[3] = {0x0F};

			adapter.initialize(to_int(argv[1]), buf, 1);

			I2cMaster2::startBlocking(&adapter);

		}

		else if(cmp(argv[0], "gyro")) {

			uint8_t wr_buf[3] = {to_int(argv[1])};
			uint8_t rd_buf[3] = {0};

			xpcc::I2cWriteReadAdapter adapter;

			adapter.initialize(0x6A, wr_buf, 1, rd_buf, 1);

			I2cMaster2::startBlocking(&adapter);

			XPCC_LOG_DEBUG .printf("rd %x", rd_buf[0]);
		}


		else if(cmp(argv[0], "i2r")) {
			uint8_t addr = to_int(argv[1]);
			uint8_t reg = to_int(argv[2]);

			uint8_t buf[3];
			buf[0] = reg;

			xpcc::I2cWriteReadAdapter adapter;
			adapter.initialize(addr, buf, 1, buf, 1);

			I2cMaster2::startBlocking(&adapter);

			XPCC_LOG_DEBUG .printf("status:%d, value: %x\n", I2cMaster2::getErrorState(), buf[0]);

		}
		else if(cmp(argv[0], "i2w")) {
			uint8_t addr = to_int(argv[1]);
			uint8_t reg = to_int(argv[2]);
			uint8_t val = to_int(argv[2]);

			uint8_t buf[3];
			buf[0] = reg;
			buf[1] = val;

			xpcc::I2cWriteReadAdapter adapter;
			adapter.initialize(addr, buf, 2, buf, 0);

			I2cMaster2::startBlocking(&adapter);

			XPCC_LOG_DEBUG .printf("status:%d, value: %x\n", I2cMaster2::getErrorState(), buf[0]);

		}

		else if(cmp(argv[0], "scan")) {

			static xpcc::I2cWriteAdapter adapter;

			static uint8_t buf[3] = {0x0F};

			XPCC_LOG_DEBUG << "Scanning i2c bus\n";
			for(int i = 0; i < 128; i++) {
				adapter.initialize(i, buf, 1);
				I2cMaster2::startBlocking(&adapter);

				if(I2cMaster2::getErrorState() != xpcc::I2cMaster::Error::AddressNack) {
					XPCC_LOG_DEBUG .printf("Found device @ %x\n", i);
				}
			}
		}


		else if(cmp(argv[0], "flash")) {
			LPC_WDT->WDFEED = 0x56;
		}
	}

};

CmdTerminal cmd(device);
CmdTerminal ucmd(uart);


class Median3Vec {
	xpcc::filter::Median<float, 3> x;
	xpcc::filter::Median<float, 3> y;
	xpcc::filter::Median<float, 3> z;

public:
	void push(xpcc::Vector3f &vec) {
		x.append(vec[0]);
		y.append(vec[1]);
		z.append(vec[2]);

		x.update();
		y.update();
		z.update();
	}

	void getValue(xpcc::Vector3f &vec) {
		vec[0] = x.getValue();
		vec[1] = y.getValue();
		vec[2] = z.getValue();
	}
};

template<typename Filter>
class VecFilter {
	Filter x;
	Filter y;
	Filter z;

public:
	void append(xpcc::Vector3f &vec) {
		x.append(vec[0]);
		y.append(vec[1]);
		z.append(vec[2]);

	}

	void getValue(xpcc::Vector3f &vec) {
		vec[0] = x.getValue();
		vec[1] = y.getValue();
		vec[2] = z.getValue();
	}
};


class SensorProcessor : TickerTask {
	xpcc::Vector3f vGyro;
	xpcc::Vector3f vAcc;

	xpcc::Vector3f vGyroOffset;

	Median3Vec medianGyro;
	Median3Vec medianAcc;

	VecFilter<filter::IIR<float, 10000>> gyroZero;

	bool gyroCalib;

	void handleTick() {
		static PeriodicTimer<> t(50);

		static PeriodicTimer<> tRead(5);


		if(t.isExpired()) {


			XPCC_LOG_DEBUG .printf("G: %.4f %.4f %.4f ", vGyro[0], vGyro[1], vGyro[2]);
			XPCC_LOG_DEBUG .printf("A: %.4f %.4f %.4f\n", vAcc[0], vAcc[1], vAcc[2]);

		}

		if(accel.isDataAvail()) {
			accel.getXYZ(vAcc);

			medianAcc.push(vAcc);
			medianAcc.getValue(vAcc);

		}

		if(gyro.isDataAvail()) {
			gyro.getXYZ(vGyro);

			medianGyro.push(vGyro);
			medianGyro.getValue(vGyro);

			gyroZero.append(vGyro);

			vGyro -= vGyroOffset;

			gyroZero.getValue(vGyroOffset);
		}



		static uint8_t read = 0;

		if(tRead.isExpired()) {
			read = 0;
		}

		//read sensors in round robin fashion
		switch(read) {
		case 0:
			if(gyro.read()) {
				read++;
			}
			break;
		case 1:
			if(accel.read()) {
				read++;
			}
			break;

		}

	}

};

SensorProcessor sensors;


extern "C" void I2C2_IRQHandler() {
	lpc17::I2cMaster2::IRQ();
}

class Test : TickerTask {

	void handleTick() {
		static PeriodicTimer<> t(500);

		if(t.isExpired()) {
			LPC_WDT->WDFEED = 0xAA;
			LPC_WDT->WDFEED = 0x55;
			ledRed::toggle();
		}
	}
};

Test task;

int main() {
	//debugIrq = true;
	ledRed::setOutput(false);

	lpc17::SysTickTimer::enable();
	lpc17::SysTickTimer::attachInterrupt(sysTick);

	xpcc::Random::seed();

	Pinsel::setFunc(0, 10, 2);
	Pinsel::setFunc(0, 11, 2);


	lpc17::I2cMaster2::initialize<xpcc::I2cMaster::DataRate::Fast>();

	accel.initialize(0x1C);
	gyro.initialize(0x6A);

	usbConnPin::setOutput(true);
	device.connect();

	TickerTask::tasksRun();
}

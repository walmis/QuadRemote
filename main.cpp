/*
 * main.cpp
 *
 *  Created on: Feb 28, 2013
 *      Author: walmis
 */
#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>


#include <lpc17xx_nvic.h>
#include <lpc17xx_uart.h>
#include <lpc17xx_pinsel.h>


#include <xpcc/driver/connectivity/wireless/at86rf230.hpp>

#include <xpcc/debug.hpp>
#include <xpcc/math/filter.hpp>

#include <xpcc/driver/connectivity/usb/USBDevice.hpp>
#include <xpcc/io/terminal.hpp>

#include <math.h>
#include <new>

#include "pindefs.hpp"

#include "SensorProcessor.hpp"

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "QuadV0.01";


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

#define M_FRONT_LEFT 1
#define M_FRONT_RIGHT 0
#define M_REAR_LEFT 2
#define M_REAR_RIGHT 3



class QuadController : TickerTask {

public:

	QuadController(SensorProcessor& sensors) : sensors(sensors),
		rollController(0, 0, 0, 0, 1.0f),
		pitchController(0, 0, 0, 0, 1.0f)
	{

		Pinsel::setFunc(2,2,1);
		Pinsel::setFunc(2,3,1);
		Pinsel::setFunc(2,4,1);
		Pinsel::setFunc(2,5,1);

		PWM::initTimer(10);

		const int freq = 100;
		prescale = SystemCoreClock / 10 / freq;

		PWM::matchUpdate(0, prescale);

		PWM::configureMatch(0, PWM::MatchFlags::RESET_ON_MATCH);

		PWM::channelEnable(3);
		PWM::channelEnable(4);
		PWM::channelEnable(5);
		PWM::channelEnable(6);

		PWM::matchUpdate(3, 0);
		PWM::matchUpdate(4, 0);
		PWM::matchUpdate(5, 0);
		PWM::matchUpdate(6, 0);

		PWM::enable();

	}

	struct DataPkt {
		struct {
			float w;
			float x;
			float y;
			float z;
		} rotationQ;
		struct {
			float x;
			float y;
			float z;
		} gyro;
		struct {
			float x;
			float y;
			float z;
		} acc;
		struct {
			float x;
			float y;
			float z;
		} mag;
		struct {
			float yaw;
			float pitch;
			float roll;
		} yawPitchRoll;
		struct {
			float x;
			float y;
			float z;
		} dynamicAcc;
		struct {
			float x;
			float y;
			float z;
		} velocity;
		struct {
			float a;
			float b;
			float c;
			float d;
		} pwm;
	} __attribute__((packed));

	void fillPacket(DataPkt &packet) {
		packet.rotationQ.w = sensors.qRotation.w;
		packet.rotationQ.x = sensors.qRotation.x;
		packet.rotationQ.y = sensors.qRotation.y;
		packet.rotationQ.z = sensors.qRotation.z;

		packet.gyro.x = sensors.vGyro.x;
		packet.gyro.x = sensors.vGyro.x;
		packet.gyro.x = sensors.vGyro.x;

		packet.acc.x = sensors.vAcc.x;
		packet.acc.y = sensors.vAcc.y;
		packet.acc.z = sensors.vAcc.z;

		packet.mag.x = sensors.vMag.x;
		packet.mag.y = sensors.vMag.y;
		packet.mag.z = sensors.vMag.z;

		packet.dynamicAcc.x = sensors.dynamicAcc.x;
		packet.dynamicAcc.y = sensors.dynamicAcc.y;
		packet.dynamicAcc.z = sensors.dynamicAcc.z;

		xpcc::Vector3f angles = sensors.getYawPitchRoll();
		packet.yawPitchRoll.yaw = angles[0];
		packet.yawPitchRoll.pitch = angles[1];
		packet.yawPitchRoll.roll = angles[2];

		packet.velocity.x = sensors.velocity.x;
		packet.velocity.y = sensors.velocity.y;
		packet.velocity.z = sensors.velocity.z;

		packet.pwm.a = motorSpeeds[0];
		packet.pwm.b = motorSpeeds[1];
		packet.pwm.c = motorSpeeds[2];
		packet.pwm.d = motorSpeeds[3];

	}

	Pid<float, 1> rollController;
	Pid<float, 1> pitchController;

	void setPidPitchRoll(float Kp, float Ki, float Kd) {
		const float dt = 0.01;
		Kp *= dt;
		Kd *= dt;
		Ki *= dt;

		Pid<float, 1>::Parameter p(Kp, Ki, Kd, 10.0, 1.0);

		rollController.setParameter(p);
		pitchController.setParameter(p);

	}

	void handleTick() override {
		static PeriodicTimer<> t(100);
		static PeriodicTimer<> tControl(10);

		if(tControl.isExpired() && pidEnable) {
			float motors[4];

			xpcc::Vector3f angles = sensors.getYawPitchRoll();
			float roll = angles[1];
			float pitch = angles[2];
			//qRotation

			//stream.printf("%.4f %.4f\n", roll, pitch);

			float targetRoll = 0;
			float targetPitch = 0;

			rollController.update((targetRoll - roll));
			pitchController.update((targetPitch - pitch));

			float pitchSet = pitchController.getValue();
			float rollSet = rollController.getValue();

			//stream.printf("pid %.4f\n", rollSet);

			const float minSpeed = 0.05f;

			motors[0] = throttle;
			motors[1] = throttle;
			motors[2] = throttle;
			motors[3] = throttle;

			motors[M_FRONT_LEFT] -= rollSet;
			motors[M_REAR_LEFT] -= rollSet;

			motors[M_FRONT_RIGHT] += rollSet;
			motors[M_REAR_RIGHT] += rollSet;

			motors[M_FRONT_LEFT] += pitchSet;
			motors[M_FRONT_RIGHT] += pitchSet;

			motors[M_REAR_LEFT] -= pitchSet;
			motors[M_REAR_RIGHT] -= pitchSet;

			for(int i =0; i < 4; i++) {
				if(motors[i] > 1.0f) motors[i] = 1.0f;
				else if(motors[i] < minSpeed) motors[i] = minSpeed;
			}

			//stream.printf("motors (%.2f, %.2f, %.2f, %.2f)\n", motors[0], motors[1], motors[2], motors[3]);

			setMotorOutput(motors);

		}


		if(t.isExpired()) {

			//XPCC_LOG_DEBUG << "Acc" << sensors.vAcc << "\n";

			auto a = sensors.vAcc;
			Quaternion<float> q;

			auto v = Vector3f(-0.439,0.137,0.889);
			auto g = Vector3f(0,0,1);

			auto cross = v.cross(g);
			//XPCC_LOG_DEBUG << "c" << cross << "\n";

			q.x = cross.x;
			q.y = cross.y;
			q.z = cross.z;
			q.w = sqrtf(v.getLengthSquared() * g.getLengthSquared()) + v * g;
			q.normalize();


			//q.x = a.x;
			//q.y = a.y;
			//q.z = a.z;

			//q.normalize();
			//XPCC_LOG_DEBUG << "q" << q << "\n";
			//sensors.qRotation = q;

			//XPCC_LOG_DEBUG << "rot" << a.rotated(q) << "\n";

			if(streamData) {
				stream.write(0xAA);

				DataPkt packet;
				fillPacket(packet);

				stream.write((uint8_t*)&packet, sizeof(packet));

				stream.write(0x55);
			}



//			XPCC_LOG_DEBUG .printf("G: %.4f %.4f %.4f ", xpcc::Angle::toDegree(vGyro[0]),
//					xpcc::Angle::toDegree(vGyro[1]),
//					xpcc::Angle::toDegree(vGyro[2]));
//
//			XPCC_LOG_DEBUG .printf("A: %.4f %.4f %.4f\n", vAcc[0], vAcc[1], vAcc[2]);

//			Vector3f noGravity = gravityCompensateAcc(vAcc, qRotation);
//			XPCC_LOG_DEBUG .printf("x:%.4f y:%.4f z:%.4f ", dynamicAcc[0],
//					dynamicAcc[1], dynamicAcc[2]);
//
//			XPCC_LOG_DEBUG .printf("vx:%.4f vy:%.4f vz:%.4f\n", velocity[0],
//					velocity[1], velocity[2]);
//
//			XPCC_LOG_DEBUG .printf("sz: %d\n", sizeof(DataPkt));



		}

	}
/*
	 ---       ---
	| 1 |     | 0 |
	 ---   ^   ---
		\  ^ /
		 \  /
		 /  \
		/    \
	 ---      ---
	| 2 |    | 3 |
	 ---      ---
*/
	void setMotorOutput(float speeds[4]) {
		int min = prescale / 10; //1ms pulsewidth = 0% motor output
		int max = prescale / 5; //2ms pulsewidth = 100% motor output
		max /= 2;

		memcpy(motorSpeeds, speeds, sizeof(float)*4);

		//XPCC_LOG_DEBUG .printf("%.2f %.2f %.2f %.2f\n", speeds[0], speeds[1], speeds[2], speeds[3]);

		auto m = PWM::multiMatchUpdate();
		m.set(3, min + max * speeds[0]);
		m.set(4, min + max * speeds[1]);
		m.set(5, min + max * speeds[2]);
		m.set(6, min + max * speeds[3]);
		m.commit(PWM::UpdateType::PWM_MATCH_UPDATE_NEXT_RST);

//		PWM::matchUpdate(3, min + max * speeds[0]);
//		PWM::matchUpdate(4, min + max * speeds[1]);
//		PWM::matchUpdate(5, min + max * speeds[2]);
//		PWM::matchUpdate(6, min + max * speeds[3]);
	}

	bool streamData;
	bool pidEnable;

	float throttle;

	SensorProcessor& sensors;

private:
	uint32_t prescale;

	float motorSpeeds[4];
};

SensorProcessor sensors;
QuadController qController(sensors);

class CmdTerminal : public Terminal {
public:
	CmdTerminal(IODevice& device) : Terminal(device), ios(device) {

	};

protected:
	xpcc::IOStream ios;

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

		else if(cmp(argv[0], "startStream")) {
			qController.streamData = true;
		}
		else if(cmp(argv[0], "stopStream")) {
			qController.streamData = false;
		}

		else if(cmp(argv[0], "pid")) {

			float kp = 0;
			float kd = 0;
			float ki = 0;

			float_scan(argv[1], &kp);
			float_scan(argv[2], &ki);
			float_scan(argv[3], &kd);

			stream.printf("PID Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd);

			qController.setPidPitchRoll(kp, ki, kd);

		}
		else if(cmp(argv[0], "i2r")) {
			uint8_t addr = to_int(argv[1]);
			uint8_t reg = to_int(argv[2]);

			uint8_t buf[3];
			buf[0] = reg;

			xpcc::I2cWriteReadAdapter adapter;
			adapter.initialize(addr, buf, 1, buf, 1);

			I2cMaster2::startBlocking(&adapter);

			ios.printf("status:%d, value: %x\n", I2cMaster2::getErrorState(), buf[0]);

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

			ios.printf("status:%d, value: %x\n", I2cMaster2::getErrorState(), buf[0]);

		}

		else if(cmp(argv[0], "throttle")) {
			float t = to_int(argv[1]) / 100.0;
			qController.throttle = t;
		}

		else if(cmp(argv[0], "control")) {
			if(cmp(argv[1], "enable")) {
				qController.pidEnable = true;
				ios.printf("PID control enabled\n");
			} else {
				ios.printf("PID control disabled\n");
				qController.pidEnable = false;
			}
		}

		else if(cmp(argv[0], "speed")) {
			float spd[4];
			spd[0] = to_int(argv[1]) / 100.0;
			spd[1] = to_int(argv[2]) / 100.0;
			spd[2] = to_int(argv[3]) / 100.0;
			spd[3] = to_int(argv[4]) / 100.0;

			ios.printf("Motor speed (%.2f,%.2f,%.2f,%.2f)\n", spd[0], spd[1], spd[2], spd[3]);
			qController.setMotorOutput(spd);
		}

		else if(cmp(argv[0], "zero")) {
			ios.printf("Zeroing sensors\n");
			qController.sensors.zero();

		}

		else if(cmp(argv[0], "scan")) {

			xpcc::I2cWriteAdapter adapter;

			uint8_t buf[3] = {0x0F};

			XPCC_LOG_DEBUG << "Scanning i2c bus\n";
			for(int i = 0; i < 128; i++) {
				adapter.initialize(i, buf, 1);
				I2cMaster2::startBlocking(&adapter);

				if(I2cMaster2::getErrorState() != xpcc::I2cMaster::Error::AddressNack) {
					XPCC_LOG_DEBUG .printf("Found device @ %x\n", i);
				}
			}
		}
		else if(cmp(argv[0], "reset")) {
			NVIC_SystemReset();
		}

		else if(cmp(argv[0], "flash")) {
			LPC_WDT->WDFEED = 0x56;
		}
	}

};

CmdTerminal cmd(device);
CmdTerminal ucmd(uart);


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

rf230::Driver<SpiMaster0, radioRst, radioSel, radioSlpTr, radioIrq> radio;


int main() {
	//debugIrq = true;
	ledRed::setOutput(false);

	lpc17::SysTickTimer::enable();
	lpc17::SysTickTimer::attachInterrupt(sysTick);

	SpiMaster0::initialize(SpiMaster0::Mode::MODE_3, 8000000);

	xpcc::Random::seed();

	Pinsel::setFunc(0, 10, 2);
	Pinsel::setFunc(0, 11, 2);

	lpc17::I2cMaster2::initialize<xpcc::I2cMaster::DataRate::Fast>();

	radio.init();



	sensors.initialize();

	usbConnPin::setOutput(true);
	device.connect();

	TickerTask::tasksRun();
}

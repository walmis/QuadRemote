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

#include "SensorProcessor.hpp"
#include "ControllerInputs.hpp"

#include "eedata.hpp"

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "QuadV0.3";


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

#define M_FRONT_LEFT 1
#define M_FRONT_RIGHT 0
#define M_REAR_LEFT 2
#define M_REAR_RIGHT 3

class Blinker : TickerTask {
	PeriodicTimer<> t;
public:
	Blinker() : t(800) {}

	enum {
		STATE_WAITING,
		STATE_SIGNAL,
		STATE_ARMED
	};

	void handleTick() override {
		if(t.isExpired()) {
			ledRed::toggle();
		}
	}

public:
	void setState(int state) {
		switch(state) {
		case STATE_WAITING:
			t.restart(800);
			break;
		case STATE_SIGNAL:
			t.restart(300);
			break;
		case STATE_ARMED:
			t.restart(100);
			break;
		}
	}
};

Blinker blinker;
#include <stdfix.h>
class QuadController : public SensorProcessor {

public:

	QuadController() :
		rollController(0, 0, 0, 0, 1.0f),
		pitchController(0, 0, 0, 0, 1.0f),
		qTarget(1.0, 0, 0, 0)
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
		struct {float w;float x;float y;float z;} rotationQ;
		struct {float w;float x;float y;float z;} targetQ;
		struct {float x;float y;float z;} gyro;
		struct {float x;float y;float z;} acc;
		struct {float x;float y;float z;} mag;
		struct {float x;float y;float z;} dynamicAcc;
		struct {float x;float y;float z;} velocity;
		struct {uint8_t a;uint8_t b;uint8_t c;uint8_t d;} pwm;
		float height;
		uint8_t throttle;
	} __attribute__((packed));

	void fillPacket(DataPkt &packet) {
		packet.rotationQ.w = qRotation.w;
		packet.rotationQ.x = qRotation.x;
		packet.rotationQ.y = qRotation.y;
		packet.rotationQ.z = qRotation.z;

		packet.targetQ.w = qTarget.w;
		packet.targetQ.x = qTarget.x;
		packet.targetQ.y = qTarget.y;
		packet.targetQ.z = qTarget.z;

		packet.gyro.x = vGyro.x;
		packet.gyro.y = vGyro.y;
		packet.gyro.z = vGyro.z;

		packet.acc.x = vAcc.x;
		packet.acc.y = vAcc.y;
		packet.acc.z = vAcc.z;

		packet.mag.x = vMag.x;
		packet.mag.y = vMag.y;
		packet.mag.z = vMag.z;

		packet.dynamicAcc.x = dynamicAcc.x;
		packet.dynamicAcc.y = dynamicAcc.y;
		packet.dynamicAcc.z = dynamicAcc.z;

		packet.velocity.x = velocity.x;
		packet.velocity.y = velocity.y;
		packet.velocity.z = velocity.z;

		packet.pwm.a = motorSpeeds[0]*255.0;
		packet.pwm.b = motorSpeeds[1]*255.0;
		packet.pwm.c = motorSpeeds[2]*255.0;
		packet.pwm.d = motorSpeeds[3]*255.0;

		packet.height = height;
		packet.throttle = (throttle + throttleComp)*255.0;
	}

	void handleInit() override {

		if(eeprom.isValidToken()) {

			float pid[3];
			float hpid[4];

			XPCC_LOG_DEBUG .printf("loading PID params\n");
			eeprom.eeRead(EEData::PIDParams, pid);
			setPID(pid[0], pid[1], pid[2], false);

			eeprom.eeRead(EEData::ratePIDParams, pid);
			setRatePID(pid[0], pid[1], pid[2], false);

			eeprom.eeRead(EEData::heightPIDparams, hpid);
			setHeightPID(hpid[0], hpid[1], hpid[2], hpid[3], false);

			eeprom.eeRead(EEData::yawGain, yawGain);

		} else {

			const float rpid[3] = {0.1070, 0.0015, 0};
			const float pid[3]  = {1.4700, 0.0700, 0};
			const float hpid[4] = {0, 0, 0.25, 0.2};

			setPID(pid[0], pid[1], pid[2]);
			setRatePID(rpid[0], rpid[1], rpid[2]);
			setHeightPID(hpid[0], hpid[1], hpid[2], hpid[3]);

			eeprom.eeWrite(EEData::yawGain, yawGain);
		}

		this->SensorProcessor::handleInit();
	}

	bool isArmed() {
		return pidEnable;
	}

	void handleTick() override {
		static PeriodicTimer<> updateTimer(10);

		if(updateTimer.isExpired()) {
			static PeriodicTimer<> tPrint(200);
			static Vector3f v;

//			if(pwmInputs.isActive()) {
//
//				if(tPrint.isExpired()) {
//					XPCC_LOG_DEBUG .printf("y:%.2f p:%.2f r:%.2f t:%.2f %d\n", pwmInputs.getYaw(),
//							pwmInputs.getPitch(), pwmInputs.getRoll(),
//							pwmInputs.getThrottle(), pwmInputs.getAux());
//
//					//XPCC_LOG_DEBUG .printf("h %.3f\n", height);
//					//XPCC_LOG_DEBUG << "Target " << qTarget << endl;
//				}
//
//				if(pwmInputs.getAux()) {
//
//					if(!isArmed()) {
//						if(pwmInputs.getThrottle() < 0.05) {
//							arm(true);
//						}
//					}
//					static Vector3f v;
//
//					v[0] -= pwmInputs.getYaw() * (45 * M_PI/180.0 * 0.010);
//					v[1] = pwmInputs.getPitch() * (15 * M_PI/180.0);
//					v[2] = pwmInputs.getRoll() * (15 * M_PI/180.0);
//
//					qTarget = Quaternion<float>(v[0], v[1], v[2]);
//
//					throttle = pwmInputs.getThrottle();
//
//				} else {
//					if(isArmed() && pwmInputs.getThrottle() < 0.05) {
//						arm(false);
//					}
//				}
//
//			}
//			else {
//				XPCC_LOG_DEBUG .printf("!");
//
//				qTarget = Quaternion<float>(v[0], 0, 0);
//
//			}


		}
		this->SensorProcessor::handleTick();
	}

	void setPID(float Kp, float Ki, float Kd, bool store = true) {
		Pid<float, 1>::Parameter p(Kp, Ki, Kd, 5.0, 20.0);

		rollController.setParameter(p);
		pitchController.setParameter(p);
		yawController.setParameter(p);

		if(store) {
			float pid[3] = {Kp,Ki,Kd};
			eeprom.eeWrite(EEData::PIDParams, pid);
		}
	}

	void setHeightPID(float Kp, float Ki, float Kd, float maxOutput = 0.2, bool store = true) {
		Pid<float, 1>::Parameter p(Kp, Ki, Kd, 5.0, maxOutput);

		heightController.setParameter(p);

		if(store) {
			float pid[4] = {Kp, Ki, Kd, maxOutput};
			eeprom.eeWrite(EEData::heightPIDparams, pid);
		}

	}

	void setRatePID(float Kp, float Ki, float Kd, bool store = true) {
		Pid<float, 1>::Parameter p(Kp, Ki, Kd, 1.0, 1.0);

		rollRateController.setParameter(p);
		pitchRateController.setParameter(p);
		yawRateController.setParameter(p);

		if(store) {
			float pid[3] = {Kp,Ki,Kd};
			eeprom.eeWrite(EEData::ratePIDParams, pid);
		}
	}

	void arm(bool armed) {
		XPCC_LOG_DEBUG .printf("ARM %d\n", armed);
		rollController.reset();
		yawController.reset();
		pitchController.reset();
		rollRateController.reset();
		pitchRateController.reset();
		yawRateController.reset();
		heightController.reset();

		throttle = 0;
		throttleComp = 0;

		if(armed) {
			pidEnable = true;
			blinker.setState(Blinker::STATE_ARMED);
		} else {
			pidEnable = false;
			blinker.setState(Blinker::STATE_SIGNAL);

			stopMotors = true;
		}
	}



	void updateRatePID(float dt) {
		Vector3f error = vGyro - targetVAngular;

		rollRateController.update(error.y, dt);
		pitchRateController.update(error.x, dt);
		yawRateController.update(error.z, dt);

		float pitchSet = pitchRateController.getValue();
		float rollSet = rollRateController.getValue();
		float yawSet = yawRateController.getValue() * yawGain;

		//XPCC_LOG_DEBUG .printf("err sum %.4f\n", rollController.getErrorSum());

		//stream.printf("pid %.4f\n", rollSet);

		const float minSpeed = 0.05f;

		float t = throttle + throttleComp;

		float motors[4] = {t,t,t,t};

		motors[M_FRONT_RIGHT] += yawSet;
		motors[M_REAR_LEFT]   += yawSet;
		motors[M_FRONT_LEFT]  -= yawSet;
		motors[M_REAR_RIGHT]  -= yawSet;

		motors[M_FRONT_LEFT] -= rollSet;
		motors[M_REAR_LEFT] -= rollSet;

		motors[M_FRONT_RIGHT] += rollSet;
		motors[M_REAR_RIGHT] += rollSet;

		motors[M_FRONT_LEFT] -= pitchSet;
		motors[M_FRONT_RIGHT] -= pitchSet;

		motors[M_REAR_LEFT] += pitchSet;
		motors[M_REAR_RIGHT] += pitchSet;

		for(int i =0; i < 4; i++) {
			if(motors[i] > 1.0f) motors[i] = 1.0f;
			else if(motors[i] < minSpeed) motors[i] = minSpeed;
		}

		setMotorOutput(motors);
	}

	void onSensorsCalibrated() override {
		float spd[] = {0,0,0,0};
		setMotorOutput(spd);
	}

	void updatePID(float dt) {
		float z0 = qRotation.w*-qTarget.z + qRotation.z*qTarget.w + qRotation.x*-qTarget.y - qRotation.y*-qTarget.x;
		float w0 = qRotation.w*qTarget.w - qRotation.x*-qTarget.x - qRotation.y*-qTarget.y - qRotation.z*-qTarget.z;

		float eyaw = 2 * atanf(z0 / w0);

		Vector3f err = getGravity(qRotation).cross(getGravity(qTarget));

		err.x = asinf(err.x);
		err.y = asinf(err.y);

		//XPCC_LOG_DEBUG << "err " << err << "eyaw: " << eyaw << endl;

		//ex = 2*atanf(qError.x/qError.w);
		//ey = 2*atanf(qError.y/qError.w);
		//XPCC_LOG_DEBUG .printf("x:%.4f y:%.4f ez:%.4f y:%.4f\n", ex, ey, ez, eyaw);

		//XPCC_LOG_DEBUG .printf ("%.3f %.3f %.3f y:%.3f\n", m[2][0], m[2][1], m[2][2], yaw);

//		Matrix3f m;
//		qError.to3x3Matrix(&m);
//		XPCC_LOG_DEBUG << "matrix\n" << m << endl;

		//Vector3f verr = Vector3f(qError.x, qError.y, qError.z);

		//Vector3f gravity = getGravity(qRotation).rotated(qError);

		//Vector3f err = getYawPitchRoll(qRotation, gravity);

		//XPCC_LOG_DEBUG << "err " << err << endl;

		//take yaw error from quaternion
		//err.x = qError.z;

		pitchController.update(-err.x, dt);
		rollController.update(-err.y, dt);
		yawController.update(-eyaw, dt);

		Vector3f v(pitchController.getValue(),
				rollController.getValue(),
				yawController.getValue());

		targetVAngular = v;

	}

	void onHeightUpdated(float dt) {
		if(isArmed()) {
			heightController.update(-height, dt);

			throttleComp = heightController.getValue();

			//XPCC_LOG_DEBUG .printf("h pid %.3f %.3f\n", height, heightController.getValue());
		}
	}

	void onSensorsUpdated(float dt) override {
		if(isArmed()) {
			updateRatePID(dt);
			updatePID(dt);
		} else {
			if(stopMotors) {
				const float mot[4] = {0,0,0,0};
				setMotorOutput(mot);
				stopMotors = false;
			}
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
	void setMotorOutput(const float speeds[4]) {
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
	}

	//state variables
	volatile bool pidEnable;
	volatile bool stopMotors;

	//absolute throttle value that is used by the controllers
	float throttle;

	//throttle compensation
	float throttleComp;

	DataPkt packet;

	//desired quadrotor orientation
	Quaternion<float> qTarget;

	//desired angular rates
	Vector3f targetVAngular;

	//PID controllers
	Pid<float> rollRateController;
	Pid<float> pitchRateController;
	Pid<float> yawRateController;
	Pid<float> rollController;
	Pid<float> pitchController;
	Pid<float> yawController;
	Pid<float> heightController;

	float yawGain = 1.7;

	//Radio controller pwm inputs
	//ControllerInputs pwmInputs;

private:
	uint32_t prescale;

	float motorSpeeds[4];
};


QuadController qController;

class CmdTerminal : public Terminal {
public:
	CmdTerminal(IODevice& device) : Terminal(device), ios(device) {

	};

protected:
	xpcc::IOStream ios;

	void handleCommand(uint8_t nargs, char* argv[]) {

		if(cmp(argv[0], "pid")) {

			float kp = 0;
			float kd = 0;
			float ki = 0;

			kp = toFloat(argv[1]);
			ki = toFloat(argv[2]);
			kd = toFloat(argv[3]);

			stream.printf("PID Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd);

			qController.setRatePID(kp, ki, kd);
		}

		else if(cmp(argv[0], "hpid")) {
			float kp = 0;
			float kd = 0;
			float ki = 0;
			float max = 0.2;


			kp = toFloat(argv[1]);
			ki = toFloat(argv[2]);
			kd = toFloat(argv[3]);
			max = toFloat(argv[4]);

			stream.printf("Height PID Kp=%.3f Ki=%.3f Kd=%.3f max=%.3f\n", kp, ki, kd, max);

			qController.setHeightPID(kp, ki, kd, max, true);

		}
		else if(cmp(argv[0], "test")) {

			//XPCC_LOG_DEBUG .printf("%d\n", qController.mpu.getAccelerationZ());
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
			uint8_t val = to_int(argv[3]);

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

		else if(cmp(argv[0], "arm")) {
			if(cmp(argv[1], "true")) {
				qController.arm(true);
			} else {
				qController.arm(false);
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
			qController.zero();
		}


		else if(cmp(argv[0], "input_calib")) {
			if(cmp(argv[1], "start")) {
				ios.printf("Calibrating\n");
				//qController.pwmInputs.startCalibration();
			} else {
				ios.printf("OK\n");
				//qController.pwmInputs.stopCalibration();
			}
		}

//		else if(cmp(argv[0], "baro")) {
//			if(!qController.baro.initialize(0x77)) {
//				XPCC_LOG_DEBUG .printf("baro init failed\n");
//			} else {
//				XPCC_LOG_DEBUG .printf("baro init OK\n");
//			}
//			XPCC_LOG_DEBUG .dump_buffer((uint8_t*)&qController.baro.calReg, sizeof(qController.baro.calReg));
//		}

		else if(cmp(argv[0], "scan")) {
			xpcc::I2cWriteAdapter adapter;

			uint8_t buf[3] = {0x0F};

			XPCC_LOG_DEBUG << "Scanning i2c bus\n";
			for(int i = 0; i < 128; i++) {
				adapter.initialize(i, buf, 1);
				I2cMaster2::startBlocking(&adapter);

				if(I2cMaster2::getErrorState() != xpcc::I2cMaster::Error::AddressNack) {
					XPCC_LOG_DEBUG .printf("Found device @ 0x%x\n", i);
				}
			}
		}
		else if(cmp(argv[0], "reset")) {
			NVIC_SystemReset();
		}

		else if(cmp(argv[0], "flash")) {

			LPC_WDT->WDFEED = 0x56;
		}
		else if(cmp(argv[0], "compass_stop")) {

			qController.mag.stopCalibration();
		}
		else if(cmp(argv[0], "compass_start")) {

			qController.mag.startCalibration();
		}
		else if(cmp(argv[0], "dump")) {
			extern uint32_t crashData[3];
			if(crashData[0]) {
				XPCC_LOG_DEBUG .printf("pc  = 0x%08x\n", crashData[1]);
				XPCC_LOG_DEBUG .printf("lr  = 0x%08x\n", crashData[2]);
				delay_ms(500);
			}
		}
	}

};

CmdTerminal cmd(device);
CmdTerminal ucmd(uart);


extern "C" void I2C2_IRQHandler() {
	lpc17::I2cMaster2::IRQ();
}

rf230::Driver<SpiMaster1, radioRst, radioSel, radioSlpTr, radioIrq> radioDriver;

class SendTask;
class QuadWireless : public TinyRadioProtocol<typeof(radioDriver), AES_CCM_32> {
public:
	typedef TinyRadioProtocol<typeof(radioDriver), AES_CCM_32> Base;

	enum ReqType {
		SET_THROTTLE = USER_REQ0,
		DATA_PKT,
		SET_PID_RATE,
		SET_PID,
		ARM,
		DISARM,
		NULL_AXIS,
		SET_ORIENTATION,
		SET_TRIM,
		SET_NULLQ,
		SET_CHANNEL
	};

	QuadWireless() : Base(radioDriver) {
		zeroing = false;
	}

	void prepareBeacon(BeaconFrame& frm) override {
		strcpy(frm.name, "QuadRotor");
	}

	void requestHandler(MacFrame &frm, uint16_t address,
				uint8_t request_type, uint8_t* data, uint8_t len) {

		XPCC_LOG_DEBUG .printf("handle request %d\n", request_type);

	}

	void eventHandler(uint16_t address, EventType event) {
		if(event == EventType::ASSOCIATION_EVENT) {
			sendCalibrationData(address);
		}
	}

	void sendCalibrationData(uint16_t address) {

		struct {
			Quaternion<float> qRotationOffset;
			Quaternion<float> qTrim;
		} calibData;

		calibData.qRotationOffset = qController.qRotationOffset;
		calibData.qTrim = qController.qTrim;

		send(address,
				(uint8_t*) &calibData,
				sizeof(calibData),
				QuadWireless::NULL_AXIS, FrameType::DATA,
				TX_ACKREQ);
	}

	void dataHandler(MacFrame &frm, FrameHdr& hdr, uint16_t address,
				uint8_t *data, uint8_t len) {

		XPCC_LOG_DEBUG .printf("handle data %d\n", hdr.req_id);

		switch(hdr.req_id) {
			case SET_THROTTLE:{
				uint8_t throttle = data[0];
				//XPCC_LOG_DEBUG .printf("throttle %d\n", throttle);

				qController.throttle = data[0] / 100.0;

			}
			break;

			case SET_PID_RATE: {
				float* d = (float*)data;
				XPCC_LOG_DEBUG .printf("setratepid %.3f %.3f %.3f\n", d[0], d[1], d[2]);
				qController.setRatePID(d[0], d[1], d[2]);
			}
			break;

			case SET_PID: {
				float* d = (float*)data;
				XPCC_LOG_DEBUG .printf("setpid %.3f %.3f %.3f\n", d[0], d[1], d[2]);
				qController.setPID(d[0], d[1], d[2]);
			}
			break;

			case ARM:
				qController.arm(true);
			break;

			case DISARM: {
				qController.arm(false);
			}
			break;

			case NULL_AXIS:
				qController.zeroRotation();
				zeroing = true;
			break;

			case SET_ORIENTATION:{
				float* vals = (float*)data;

				qController.qTarget = Quaternion<float>(vals[0], vals[1], vals[2], vals[3]);
				//XPCC_LOG_DEBUG << "t " << qController.qTarget << endl;
			}
			break;

			case SET_TRIM:{
				float* vals = (float*)data;
				auto q = Quaternion<float>(0, vals[0], vals[1]);
				XPCC_LOG_DEBUG << "trim " << q << "\n";
				qController.applyTrim(q);
			}
			break;

			case SET_CHANNEL: {
				uint8_t* val = (uint8_t*)data;

				radioDriver.setChannel(*val);
			}

//			case SET_NULLQ:{
//				float* vals = (float*)data;
//				Quaternion<float> q;
//				q.w = vals[0];
//				q.x = vals[1];
//				q.y = vals[2];
//				q.z = vals[3];
//
//				qController.setRotationOffset(q);
//			}

//			case GET_CALIBRATION:
//
//				struct {
//					Quaternion<float> rotOfs;
//				} data;
//
//				data.rotOfs = qController.qRotationOffset;
//
//				send(address, (uint8_t*)&data, sizeof(data), GET_CALIBRATION);
		}

		if(!isAssociated(address)) {
			associate(address);
		}

	}
	friend class SendTask;

protected:
	bool zeroing;

};

class SendTask: public TickerTask {
public:
	SendTask(QuadWireless *q) :
			parent(q) {
	}

	QuadWireless *parent;
	void handleTick() {
		static PeriodicTimer<> t(100);

		if (parent->zeroing && qController.zeroingComplete()) {
			for (auto node : parent->connectedNodes) {
				parent->sendCalibrationData(node->address);

				parent->zeroing = false;
			}
		}

		if (t.isExpired()) {
			{
				//PROFILE();
				for (auto node : parent->connectedNodes) {
					qController.fillPacket(qController.packet);
					parent->send(node->address,
							(uint8_t*) &qController.packet,
							sizeof(qController.packet),
							QuadWireless::DATA_PKT, FrameType::DATA, 0);

				}
			}
		}
	}
};


QuadWireless radio;
SendTask sender(&radio);

GPIO__OUTPUT(test, 0, 16);

void idle() {
	test::reset();
	__WFI();
	test::set();
	static PeriodicTimer<> t(500);
	static bool eeReady;

	if(t.isExpired()) {
		if(!eeReady) {
			eeprom.setToken();
			eeReady = true;
		}

		LPC_WDT->WDFEED = 0xAA;
		LPC_WDT->WDFEED = 0x55;
	}
}

void __attribute((noinline, used)) testas() {

	Quaternion<Fp32f<29>> q;
	q.w = (int16_t)LPC_UART0->ACR;
	q.x =(int16_t)LPC_UART0->ACR;
	q.y = (int16_t)LPC_UART0->ACR;
	q.z = 0.5;
	q.normalize();

	Quaternion<Fp32f<29>> q2 = q;

	q = q * q2.conjugated();

	q.w = 1 - 2*(q.w*q.w);

	XPCC_LOG_DEBUG << "Test" << std::sqrt(q.w) << endl;

}

int main() {
	test::setOutput(false);
	//debugIrq = true;
	ledRed::setOutput(false);
	ledGreen::setOutput(false);

	lpc17::RitClock::initialize();

	lpc17::SysTickTimer::enable();
	//lpc17::SysTickTimer::attachInterrupt(sysTick);

	Pinsel::setFunc(0, 7, 2); //SCK0
	Pinsel::setFunc(0, 8, 2); //MISO0
	Pinsel::setFunc(0, 9, 2); //MOSI0

	SpiMaster1::initialize(SpiMaster1::Mode::MODE_0, 8000000);

	xpcc::Random::seed();

	testas();

	Pinsel::setFunc(0, 10, 2);
	Pinsel::setFunc(0, 11, 2);

	lpc17::I2cMaster2::initialize<xpcc::I2cMaster::DataRate::Fast>();

	//initialize eeprom
	eeprom.initialize();

	//initialize radio
	radio.init();
	radioDriver.setCLKM(rf230::no_clock);

	radio.setAddress(0x9809);
	radio.setPanId(0x0001);

	radioDriver.rxOn();

	usbConnPin::setOutput(true);
	device.connect();

	NVIC_SetPriority(USB_IRQn, 10);
	NVIC_SetPriority(EINT3_IRQn, 0);

	TickerTask::tasksRun(idle);
}

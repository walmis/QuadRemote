/*
 * ControllerInputs.hpp
 *
 *  Created on: May 7, 2014
 *      Author: walmis
 */

#ifndef CONTROLLERINPUTS_HPP_
#define CONTROLLERINPUTS_HPP_

#include "pindefs.hpp"
#include "pwmChannel.hpp"
#include "eedata.hpp"
#include <xpcc/processing.hpp>



class ControllerInputs : TickerTask {

public:
	ControllerInputs() :
			channel0(pwmch1::Port, pwmch1::Pin),
			channel1(pwmch2::Port, pwmch2::Pin),
			channel2(pwmch3::Port, pwmch3::Pin),
			channel3(pwmch4::Port, pwmch4::Pin),
			channel4(pwmch5::Port, pwmch5::Pin)
	{
		calibrating = false;
	}

	void handleInit() override {

		if(eeprom.isValidToken()) {
			eeprom.eeRead(EEData::PWMInputCalibration, calibData);
			//XPCC_LOG_DEBUG .printf("read calib data\n");
			//XPCC_LOG_DEBUG .dump_buffer((uint8_t*) calibData, sizeof(calibData));
		} else {
			eeprom.eeWrite(EEData::PWMInputCalibration, calibData);
		}

		//XPCC_LOG_DEBUG .dump_buffer((uint8_t*)calibData, sizeof(calibData));

	}

	void handleTick() override {


		if(calibrating) {
			static PeriodicTimer<> t(500);
			if(t.isExpired()) {
				XPCC_LOG_DEBUG.printf("t %d %d %d %d %d\n", readChannel(0),
						readChannel(1), readChannel(2),
						readChannel(3), readChannel(4));
			}

			for(int i = 0; i < 5; i++) {
				if(this->channels[i]->isActive()) {
					uint16_t duty = this->channels[i]->getDuty();

					if(duty > calibData[i].max) {
						calibData[i].max = duty;
					}
					if(duty < calibData[i].min) {
						calibData[i].min = duty;
					}
				}
			}
		}
	}

	void startCalibration() {
		calibrating = true;
		for(int i = 0; i < 5; i++) {
			calibData[i].max = 0;
			calibData[i].min = 0xFFFF;
		}
	}

	void stopCalibration() {
		calibrating = false;

		eeprom.eeWrite(EEData::PWMInputCalibration, calibData);
		//XPCC_LOG_DEBUG .dump_buffer((uint8_t*)calibData, sizeof(calibData));
	}

	bool isActive() {
		if(calibrating) return false;
 		for(int i = 0; i < 5; i++) {
			if(!channels[i]->isActive()) {
				return false;
			}
		}
		return true;
	}

	int16_t readChannel(uint8_t channel) {
		int16_t val = this->channels[channel]->getDuty();

		uint16_t min = calibData[channel].min;
		uint16_t max = calibData[channel].max;

		val = (val-min)*1024/(max-min);
		if(val > 1024) val = 1024;
		if(val < 0) val = 0;
		return val;
	}

	float getAxis(uint8_t channel) {
		if(channels[channel]->isActive()) {
			int duty = readChannel(channel);
			//XPCC_LOG_DEBUG .printf("%d\n", duty);
			float val = (512 - duty) / 512.0;
			if(fabs(val) < 0.05) {
				val = 0;
			}
			return val;
		}
		return 0;
	}

	float getPitch() {
		return getAxis(1);
	}

	float getRoll() {
		return getAxis(3);
	}

	float getYaw() {
		return getAxis(2);
	}

	float getThrottle() {
		if(channel4.isActive()) {
			int duty = readChannel(4);
			float v = duty / 1024.0;

			const float k = 0.2;
			return (k*v - v)/(2*k*v-k-1);
		}
		return 0;
	}

	bool getAux() {
		if(channel0.isActive()) {
			int val = readChannel(0);

			if(val > 800) {
				return false;
			}
			if(val < 100) {
				return true;
			}
		}
		return false;
	}

private:
	PWMChannel channel0;
	PWMChannel channel1;
	PWMChannel channel2;
	PWMChannel channel3;
	PWMChannel channel4;

	PWMChannel* const channels[5] = {&channel0, &channel1, &channel2, &channel3, &channel4};

	CalibrationData calibData[5];
	bool calibrating;
};




#endif /* CONTROLLERINPUTS_HPP_ */

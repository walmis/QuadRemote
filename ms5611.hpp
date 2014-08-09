/*
 * ms5611.hpp
 *
 *  Created on: Apr 18, 2014
 *      Author: walmis
 */

#ifndef MS5611_HPP_
#define MS5611_HPP_

#include <xpcc/architecture/peripheral/i2c_adapter.hpp>

/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @file ms5611.c
 * Driver for the ms5611 pressure sensor from measurement specialties.
 * Datasheet at http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
 *
 */
#define DEBUG_MODULE "MS5611"

#include <xpcc/architecture.hpp>
#include <xpcc/math/filter.hpp>

namespace xpcc {

#define DEBUG_PRINT XPCC_LOG_DEBUG .printf

// addresses of the device
#define MS5611_ADDR_CSB_HIGH  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS5611_ADDR_CSB_LOW   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)

// registers of the device
#define MS5611_D1 0x40
#define MS5611_D2 0x50
#define MS5611_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS5611_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 0x00
#define MS5611_OSR_512 0x02
#define MS5611_OSR_1024 0x04
#define MS5611_OSR_2048 0x06
#define MS5611_OSR_4096 0x08
#define MS5611_OSR_DEFAULT MS5611_OSR_4096

#define MS5611_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values.
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS5611_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS5611_PROM_REG_SIZE 2 // size in bytes of a prom registry.

// Self test parameters. Only checks that values are sane
#define MS5611_ST_PRESS_MAX   (1100.0) //mbar
#define MS5611_ST_PRESS_MIN   (450.0)  //mbar
#define MS5611_ST_TEMP_MAX    (60.0)   //degree celcius
#define MS5611_ST_TEMP_MIN    (-20.0)  //degree celcius

// Constants used to determine altitude from pressure
#define CONST_SEA_PRESSURE 102610.f //1026.1f //http://www.meteo.physik.uni-muenchen.de/dokuwiki/doku.php?id=wetter:stadt:messung
#define CONST_PF 0.1902630958 //(1/5.25588f) Pressure factor
#define CONST_PF2 44330.0f

#include "math.h"

#define EXTRA_PRECISION      5 // trick to add more precision to the pressure and temp readings
#define CONVERSION_TIME_MS   10 // conversion time in milliseconds. 10 is minimum
#define PRESSURE_PER_TEMP    5 // Length of reading cycle: 1x temp, rest pressure. Good values: 1-10
#define FIX_TEMP 25         // Fixed Temperature. ASL is a function of pressure and temperature, but as the temperature changes so much (blow a little towards the flie and watch it drop 5 degrees) it corrupts the ASL estimates.
// TLDR: Adjusting for temp changes does more harm than good.

template <int statesize, int decimationFactor>
class CICFilter {
	int integrator = 0;
	int downsample_count = 0;
	int ringbuffer[statesize];
	// don't forget to initialize the ringbuffer somehow
	int ringbuffer_ptr = 0;
	int outstate = 0;

public:
	int32_t getValue() {
		return outstate * decimationFactor;
	}

	void append(int32_t in) {

	   integrator += in;
	   if (++downsample_count >= decimationFactor)
	   {
	     int oldintegrator = ringbuffer[ringbuffer_ptr];
	     ringbuffer[ringbuffer_ptr] = integrator;
	     ringbuffer_ptr = (ringbuffer_ptr + 1) % statesize;
	     outstate = (integrator - oldintegrator) / (statesize * decimationFactor);
	   }
	}
};

template <typename i2cMaster>
class MS5611: TickerTask {
public:
	struct {
		uint16_t psens;
		uint16_t off;
		uint16_t tcs;
		uint16_t tco;
		uint16_t tref;
		uint16_t tsens;
	} calReg;

	bool isInit;

	uint8_t readState = 0;

	float pressure;
	float temperature;

	int32_t tempDeltaT;

	I2cWriteReadAdapter adapter;

	//CICFilter<2, 32> pressFilter;
	xpcc::MovingAverage<int32_t, 32> pressFilter;

	uint8_t buffer[MS5611_D1D2_SIZE];

	bool initialize(uint8_t address) {
		if (isInit)
			return true;

		adapter.initialize(address, 0, 0, 0, 0);

		reset(); // reset the device to populate its internal PROM registers

		delay_ms(5);

		if (ms5611ReadPROM() == false) {  // reads the PROM into object variables for later use
			return false;
		}

		isInit = true;

		return true;
	}

	bool selfTest(void) {
		bool testStatus = true;
		int32_t rawPress;
		int32_t rawTemp;
		int32_t deltaT;
		float pressure;
		float temperature;

		if (!isInit)
			return false;

		startConversion(MS5611_D1 + MS5611_OSR_4096);

		xpcc::delay_ms(CONVERSION_TIME_MS);

		rawPress = getConversion();

		startConversion(MS5611_D2 + MS5611_OSR_4096);

		xpcc::delay_ms(CONVERSION_TIME_MS);

		rawTemp = getConversion();

		deltaT = calcDeltaTemp(rawTemp);
		temperature = calcTemp(deltaT);
		pressure = calcPressure(rawPress, deltaT);

		if (evaluateSelfTest(MS5611_ST_PRESS_MIN, MS5611_ST_PRESS_MAX,
				pressure, "pressure")
				&& evaluateSelfTest(MS5611_ST_TEMP_MIN,
						MS5611_ST_TEMP_MAX, temperature, "temperature")) {
			DEBUG_PRINT("Self test [OK].\n");
		} else {
			testStatus = false;
		}

		return testStatus;
	}

	bool evaluateSelfTest(float min, float max, float value,
			char* string) {
		if (value < min || value > max) {
			DEBUG_PRINT(
					"Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
					string, min, max, value);
			return false;
		}
		return true;
	}

//	float getPressure(uint8_t osr) {
//		// see datasheet page 7 for formulas
//		int32_t rawPress = rawPressure(osr);
//		int64_t dT = (int64_t) getDeltaTemp(osr);
//		if (dT == 0) {
//			return 0;
//		}
//		int64_t off = (((int64_t) calReg.off) << 16) + ((calReg.tco * dT) >> 7);
//		int64_t sens = (((int64_t) calReg.psens) << 15)
//				+ ((calReg.tcs * dT) >> 8);
//		if (rawPress != 0) {
//			return ((((rawPress * sens) >> 21) - off) >> (15 - EXTRA_PRECISION))
//					/ ((1 << EXTRA_PRECISION) * 100.0);
//		} else {
//			return 0;
//		}
//	}

	float calcPressure(int32_t rawPress, int32_t dT) {
		int64_t off;
		int64_t sens;

		if (rawPress == 0 || dT == 0) {
			return 0;
		}

		off = (((int64_t) calReg.off) << 16)
				+ ((calReg.tco * (int64_t) dT) >> 7);
		sens = (((int64_t) calReg.psens) << 15)
				+ ((calReg.tcs * (int64_t) dT) >> 8);

		return ((((rawPress * sens) >> 21) - off) >> (15 - EXTRA_PRECISION))
				/ ((1 << EXTRA_PRECISION) * 100.0);
	}

//	float getTemperature(uint8_t osr) {
//		// see datasheet page 7 for formulas
//		int32_t dT;
//
//		dT = getDeltaTemp(osr);
//		if (dT != 0) {
//			return calcTemp(dT);
//		} else {
//			return 0;
//		}
//	}

//	int32_t getDeltaTemp(uint8_t osr) {
//		int32_t rawTemp = rawTemperature(osr);
//		if (rawTemp != 0) {
//			return calcDeltaTemp(rawTemp);
//		} else {
//			return 0;
//		}
//	}

	float calcTemp(int32_t deltaT) {
		if (deltaT == 0) {
			return 0;
		} else {
			return (float) (((1 << EXTRA_PRECISION) * 2000)
					+ (((int64_t) deltaT * calReg.tsens) >> (23 - EXTRA_PRECISION)))
					/ ((1 << EXTRA_PRECISION) * 100.0);
		}
	}

	int32_t calcDeltaTemp(int32_t rawTemp) {
		if (rawTemp == 0) {
			return 0;
		} else {
			return rawTemp - (((int32_t) calReg.tref) << 8);
		}
	}

//	int32_t rawPressure(uint8_t osr) {
//		uint32_t now = Clock::now();
//		if (lastPresConv != 0 && (now - lastPresConv) >= CONVERSION_TIME_MS) {
//			lastPresConv = 0;
//			return getConversion(MS5611_D1 + osr);
//		} else {
//			if (lastPresConv == 0 && lastTempConv == 0) {
//				startConversion(MS5611_D1 + osr);
//				lastPresConv = now;
//			}
//			return 0;
//		}
//	}
//
//	int32_t rawTemperature(uint8_t osr) {
//		uint32_t now = Clock::now();
//		if (lastTempConv != 0 && (now - lastTempConv) >= CONVERSION_TIME_MS) {
//			lastTempConv = 0;
//			tempCache = getConversion(MS5611_D2 + osr);
//			return tempCache;
//		} else {
//			if (lastTempConv == 0 && lastPresConv == 0) {
//				startConversion(MS5611_D2 + osr);
//				lastTempConv = now;
//			}
//			return tempCache;
//		}
//	}

// see page 11 of the datasheet

	bool startConversion(uint8_t command) {
		static uint8_t c[1];
		c[0] = command;

		adapter.initialize(c, 1, 0, 0);
		return i2cMaster::start(&adapter);
	}

	bool readConversion() {
		//uint8_t buffer[MS5611_D1D2_SIZE];
		static const uint8_t cmd = 0;

		adapter.initialize(&cmd, 1, buffer, MS5611_D1D2_SIZE);

		return i2cMaster::start(&adapter);

	}

	int32_t getConversion() {
		return ((int32_t) buffer[0] << 16) | ((int32_t) buffer[1] << 8)	| buffer[2];
	}



	/**
	 * Reads factory calibration and store it into object variables.
	 */
	bool ms5611ReadPROM() {
		uint8_t buffer[MS5611_PROM_REG_SIZE];
		uint16_t* pCalRegU16 = (uint16_t*) &calReg;
		bool status = false;

		for (int i = 0; i < MS5611_PROM_REG_COUNT; i++) {
			// start read sequence
			while(isBusy());

			uint8_t cmd = MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE);

			adapter.initialize(&cmd, 1, buffer, 2);

			status = i2cMaster::startBlocking(&adapter);
			if(status) {
				pCalRegU16[i] = ((uint16_t) buffer[0] << 8) | buffer[1];
			}
		}

		return status;
	}

	inline bool isBusy() {
		return (adapter.getState() == xpcc::I2c::AdapterState::Busy);
	}

	/**
	 * Send a reset command to the device. With the reset command the device
	 * populates its internal registers with the values read from the PROM.
	 */
	void reset() {
		while(isBusy());

		const uint8_t cmd = MS5611_RESET;
		adapter.initialize(&cmd, 1, 0, 0);
		i2cMaster::startBlocking(&adapter);
	}

	void handleTick() override {
		if(!isInit) {
			return;
		}

		int32_t tempPressureRaw, tempTemperatureRaw;

		static Timeout<> conversionTimeout;

		static volatile uint8_t conversion = 0;
		static volatile uint8_t reading = 0;

		if(!conversion && !reading) {
			if(readState == PRESSURE_PER_TEMP) {
				if(startConversion(MS5611_D2 + MS5611_OSR_DEFAULT)) {
					conversion = MS5611_D2;
					conversionTimeout.restart(CONVERSION_TIME_MS);
					readState = 0;
				}
			} else {
				// cmd to read pressure
				if(startConversion(MS5611_D1 + MS5611_OSR_DEFAULT)) {
					conversion = MS5611_D1;
					conversionTimeout.restart(CONVERSION_TIME_MS);
					readState++;
				}
			}
		} else

		if(conversion && conversionTimeout.isExpired()) {

			if(readConversion()) {
				reading = conversion;
				conversion = 0;
			}
		}


		if(reading && !conversion && !isBusy()) {

			if(reading == MS5611_D1) {
				//XPCC_LOG_DEBUG .printf("D1 ");
				//XPCC_LOG_DEBUG .dump_buffer((uint8_t*)buffer, MS5611_D1D2_SIZE);
				tempPressureRaw = getConversion();

				pressFilter.append(tempPressureRaw);
				pressFilter.append(pressFilter.getValue());

				//XPCC_LOG_DEBUG .printf("%d %d\n", tempPressureRaw, pressFilter.getValue());
				pressure = calcPressure(pressFilter.getValue(), tempDeltaT);

				float alt = getAltitude();

				//XPCC_LOG_DEBUG .printf("%.5f\n", alt);

				//XPCC_LOG_DEBUG .printf("%.3f\n", pressure);

			} else

			if(reading == MS5611_D2) {
				tempTemperatureRaw = getConversion();

				tempDeltaT = calcDeltaTemp(tempTemperatureRaw);
				temperature = calcTemp(tempDeltaT);
				//XPCC_LOG_DEBUG .printf("temp %.3f\n", temp);

			}

			reading = 0;
		}




//		if (readState == 0) {
//			// read temp
//			++readState;
//			readConversion(MS5611_D2 + MS5611_OSR_DEFAULT);
//			curRead = MS5611_D2;
//
//
//		} else {
//			++readState;
//
//			readConversion(MS5611_D1 + MS5611_OSR_DEFAULT);
//			curRead = MS5611_D1;
//
//			*pressure = calcPressure(tempPressureRaw, tempDeltaT);
//			*asl = ms5611PressureToAltitude(*pressure);
//			if (readState == PRESSURE_PER_TEMP) {
//				// cmd to read temp
//				startConversion(MS5611_D2 + MS5611_OSR_DEFAULT);
//				readState = 0;
//			} else {
//				// cmd to read pressure
//				startConversion(MS5611_D1 + MS5611_OSR_DEFAULT);
//			}
//		}
	}

	/**
	 * Gets pressure, temperature and above sea level altitude estimate (asl).
	 * Best called at 100hz. For every PRESSURE_PER_TEMP-1 pressure readings temp is read once.
	 * Effective 50-90hz baro update and 50-10hz temperature update if called at 100hz.
	 */
//	void ms5611GetData(float* pressure, float* temperature, float* asl) {
//		int32_t tempPressureRaw, tempTemperatureRaw;
//
//		// Dont reader faster than we can
//		uint32_t now = Clock::now();
//		if ((now - lastConv) < CONVERSION_TIME_MS) {
//			return;
//		}
//		lastConv = now;
//
//		if (readState == 0) {
//			// read temp
//			++readState;
//			tempTemperatureRaw = getConversion(MS5611_D2 + MS5611_OSR_DEFAULT);
//			tempDeltaT = calcDeltaTemp(tempTemperatureRaw);
//			*temperature = calcTemp(tempDeltaT);
//			// cmd to read pressure
//			startConversion(MS5611_D1 + MS5611_OSR_DEFAULT);
//		} else {
//			// read pressure
//			++readState;
//			tempPressureRaw = getConversion(
//					MS5611_D1 + MS5611_OSR_DEFAULT);
//			*pressure = calcPressure(tempPressureRaw, tempDeltaT);
//			*asl = ms5611PressureToAltitude(*pressure);
//			if (readState == PRESSURE_PER_TEMP) {
//				// cmd to read temp
//				startConversion(MS5611_D2 + MS5611_OSR_DEFAULT);
//				readState = 0;
//			} else {
//				// cmd to read pressure
//				startConversion(MS5611_D1 + MS5611_OSR_DEFAULT);
//			}
//		}
//	}

//TODO: pretty expensive function. Rather smooth the pressure estimates and only call this when needed

	/**
	 * Converts pressure to altitude above sea level (ASL) in meters
	 */
	float getAltitude() {
		if (pressure > 0) {
			//return (1.f - pow(*pressure / CONST_SEA_PRESSURE, CONST_PF)) * CONST_PF2;
			//return ((pow((1015.7 / *pressure), CONST_PF) - 1.0) * (25. + 273.15)) / 0.0065;
			return ((powf((1015.7 / pressure), CONST_PF) - 1.0)
					* (FIX_TEMP + 273.15)) / 0.0065;
		} else {
			return 0;
		}
	}

};
} //namespace xpcc
#endif /* MS5611_HPP_ */

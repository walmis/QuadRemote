/*
 * L3GD20.hpp
 *
 *  Created on: Apr 5, 2014
 *      Author: walmis
 */

#ifndef L3GD20_HPP_
#define L3GD20_HPP_

#include <xpcc/architecture/peripheral/i2c_adapter.hpp>
#include <xpcc/math/geometry.hpp>
// device types

#define L3G_DEVICE_AUTO 0
#define L3G4200D_DEVICE 1
#define L3GD20_DEVICE   2


// SA0 states

#define L3G_SA0_LOW  0
#define L3G_SA0_HIGH 1
#define L3G_SA0_AUTO 2

// register addresses

#define L3G_WHO_AM_I      0x0F

#define L3G_CTRL_REG1     0x20
#define L3G_CTRL_REG2     0x21
#define L3G_CTRL_REG3     0x22
#define L3G_CTRL_REG4     0x23
#define L3G_CTRL_REG5     0x24
#define L3G_REFERENCE     0x25
#define L3G_OUT_TEMP      0x26
#define L3G_STATUS_REG    0x27

#define L3G_OUT_X_L       0x28
#define L3G_OUT_X_H       0x29
#define L3G_OUT_Y_L       0x2A
#define L3G_OUT_Y_H       0x2B
#define L3G_OUT_Z_L       0x2C
#define L3G_OUT_Z_H       0x2D

#define L3G_FIFO_CTRL_REG 0x2E
#define L3G_FIFO_SRC_REG  0x2F

#define L3G_INT1_CFG      0x30
#define L3G_INT1_SRC      0x31
#define L3G_INT1_THS_XH   0x32
#define L3G_INT1_THS_XL   0x33
#define L3G_INT1_THS_YH   0x34
#define L3G_INT1_THS_YL   0x35
#define L3G_INT1_THS_ZH   0x36
#define L3G_INT1_THS_ZL   0x37
#define L3G_INT1_DURATION 0x38
#define L3G_LOW_ODR       0x39


template<typename I2cMaster>
class L3GD20 {
public:
	void initialize(uint8_t address) {

		adapter.initialize(address, 0,0,0,0);

		// 0x0F = 0b00001111
		// Normal power mode, all axes enabled
		registerWrite(L3G_CTRL_REG1, 0x0F | (0b1011)<<4);

		//registerWrite(L3G_CTRL_REG2, 0b1001);
		//registerWrite(L3G_CTRL_REG5, (1<<4) | (1<<0));

		//set full scale
		registerWrite(L3G_CTRL_REG4, 0x01 << 4);
		readPending = false;
	}

	void registerWrite(uint8_t reg, uint8_t value) {
		while (adapter.getState() == xpcc::I2c::AdapterState::Busy)
			;

		buffer[0] = reg;
		buffer[1] = value;

		adapter.initialize(buffer, 2, 0, 0);
		I2cMaster::startBlocking(&adapter);

	}

	bool read() {
		while (adapter.getState() == xpcc::I2c::AdapterState::Busy)
			;

		buffer[0] = L3G_OUT_X_L | (1 << 7);

		adapter.initialize(buffer, 1, buffer, 6);
		if(I2cMaster::start(&adapter)) {
			readPending = true;
			return true;
		}
		return false;
	}

	bool isDataAvail() {
		return !isBusy() && readPending;
	}


	bool getXYZ(xpcc::Vector3f &vector) {
		if(!isDataAvail()) {
			return false;
		}
		int16_t aux;

		//float fs_dps = 0.00875;
		float fs_dps = 0.0175;

		for(int i = 0; i < 6; i+=2) {
			aux = 0;
			aux = buffer[i+1];
			aux <<= 8;
			aux |= buffer[i];

			vector[i>>1] = aux * fs_dps;
		}
		readPending = false;
		return true;
	}

	bool isBusy() {
		return (adapter.getState() == xpcc::I2c::AdapterState::Busy);
	}

private:
	bool readPending;
	uint8_t buffer[8];
	xpcc::I2cWriteReadAdapter adapter;

};




#endif /* L3GD20_HPP_ */

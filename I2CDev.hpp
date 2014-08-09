/*
 * i2cDev.hpp
 *
 *  Created on: Jun 2, 2014
 *      Author: walmis
 */

#ifndef I2CDEV_HPP_
#define I2CDEV_HPP_

#include <string.h>
#include <xpcc/architecture.hpp>
#include <xpcc/architecture/peripheral/i2c_adapter.hpp>

#define I2C_Master xpcc::lpc17::I2cMaster2

class I2Cdev {
    public:
        I2Cdev(){};

        /** Read a single bit from an 8-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register regAddr to read from
        * @param bitNum Bit position to read (0-7)
        * @param data Container for single bit value
        * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
        * @return Status of read operation (true = success)
        */
        static int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout = 0) {
            uint8_t b;
            uint8_t count = readByte(devAddr, regAddr, &b, timeout);
            *data = b & (1 << bitNum);
            return count;
        }

        /** Read a single bit from a 16-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register regAddr to read from
        * @param bitNum Bit position to read (0-15)
        * @param data Container for single bit value
        * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
        * @return Status of read operation (true = success)
        */
        static int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout = 0) {
            uint16_t b;
            uint8_t count = readWord(devAddr, regAddr, &b, timeout);
            *data = b & (1 << bitNum);
            return count;
        }
        /** Read multiple bits from an 8-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register regAddr to read from
        * @param bitStart First bit position to read (0-7)
        * @param length Number of bits to read (not more than 8)
        * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
        * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
        * @return Status of read operation (true = success)
        */
        static int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout = 0) {
            // 01101001 read byte
            // 76543210 bit numbers
            // xxx args: bitStart=4, length=3
            // 010 masked
            // -> 010 shifted
            uint8_t count, b;
            if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
                uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
                b &= mask;
                b >>= (bitStart - length + 1);
                *data = b;
            }
            return count;
        }
        /** Read multiple bits from a 16-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register regAddr to read from
        * @param bitStart First bit position to read (0-15)
        * @param length Number of bits to read (not more than 16)
        * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
        * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
        * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
        */
        static int8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout = 0) {
            // 1101011001101001 read byte
            // fedcba9876543210 bit numbers
            // xxx args: bitStart=12, length=3
            // 010 masked
            // -> 010 shifted
            uint8_t count;
            uint16_t w;
            if ((count = readWord(devAddr, regAddr, &w, timeout)) != 0) {
                uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
                w &= mask;
                w >>= (bitStart - length + 1);
                *data = w;
            }
            return count;
        }
        /** Read single byte from an 8-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register regAddr to read from
        * @param data Container for byte value read from device
        * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
        * @return Status of read operation (true = success)
        */
        static int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout = 0) {
            return readBytes(devAddr, regAddr, 1, data, timeout);
        }

        /** Read single word from a 16-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register regAddr to read from
        * @param data Container for word value read from device
        * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
        * @return Status of read operation (true = success)
        */
        static int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout = 0) {
            return readWords(devAddr, regAddr, 1, data, timeout);
        }


        /** Read multiple bytes from an 8-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr First register regAddr to read from
        * @param length Number of bytes to read
        * @param data Buffer to store read data in
        * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
        * @return Number of bytes read (-1 indicates failure)
        */
        static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout = 0) {
        	xpcc::I2cWriteReadAdapter adapter;

        	adapter.initialize(devAddr, &regAddr, 1, data, length);

        	while(!I2C_Master::startBlocking(&adapter));

        	return adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Idle;
        }


        /** Read multiple words from a 16-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr First register regAddr to read from
        * @param length Number of words to read
        * @param data Buffer to store read data in
        * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
        * @return Number of words read (-1 indicates failure)
        */
        static int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout) {
        	xpcc::I2cWriteReadAdapter adapter;

        	adapter.initialize(devAddr, &regAddr, 1, (uint8_t*)data, length*sizeof(uint16_t));

        	while(!I2C_Master::startBlocking(&adapter));

        	return adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Idle;
        }

        /** write a single bit in an 8-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register regAddr to write to
        * @param bitNum Bit position to write (0-7)
        * @param value New bit value to write
        * @return Status of operation (true = success)
        */
        static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
            uint8_t b;
            readByte(devAddr, regAddr, &b);
            b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
            return writeByte(devAddr, regAddr, b);
        }


        /** write a single bit in a 16-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register regAddr to write to
        * @param bitNum Bit position to write (0-15)
        * @param value New bit value to write
        * @return Status of operation (true = success)
        */
        static bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
            uint16_t w;
            readWord(devAddr, regAddr, &w);
            w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
            return writeWord(devAddr, regAddr, w);
        }

        /** Write multiple bits in an 8-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register regAddr to write to
        * @param bitStart First bit position to write (0-7)
        * @param length Number of bits to write (not more than 8)
        * @param data Right-aligned value to write
        * @return Status of operation (true = success)
        */
        static bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
            // 010 value to write
            // 76543210 bit numbers
            // xxx args: bitStart=4, length=3
            // 00011100 mask byte
            // 10101111 original value (sample)
            // 10100011 original & ~mask
            // 10101011 masked | value
            uint8_t b;
            if (readByte(devAddr, regAddr, &b) != 0) {
                uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
                data <<= (bitStart - length + 1); // shift data into correct position
                data &= mask; // zero all non-important bits in data
                b &= ~(mask); // zero all important bits in existing byte
                b |= data; // combine data with existing byte
                return writeByte(devAddr, regAddr, b);
            } else {
                return false;
            }
        }
        /** Write multiple bits in a 16-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register regAddr to write to
        * @param bitStart First bit position to write (0-15)
        * @param length Number of bits to write (not more than 16)
        * @param data Right-aligned value to write
        * @return Status of operation (true = success)
        */
        static bool writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
            // 010 value to write
            // fedcba9876543210 bit numbers
            // xxx args: bitStart=12, length=3
            // 0001110000000000 mask word
            // 1010111110010110 original value (sample)
            // 1010001110010110 original & ~mask
            // 1010101110010110 masked | value
            uint16_t w;
            if (readWord(devAddr, regAddr, &w) != 0) {
                uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
                data <<= (bitStart - length + 1); // shift data into correct position
                data &= mask; // zero all non-important bits in data
                w &= ~(mask); // zero all important bits in existing word
                w |= data; // combine data with existing word
                return writeWord(devAddr, regAddr, w);
            } else {
                return false;
            }
        }


        /** Write single byte to an 8-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register address to write to
        * @param data New byte value to write
        * @return Status of operation (true = success)
        */
        static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
            return writeBytes(devAddr, regAddr, 1, &data);
        }

        /** Write single word to a 16-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr Register address to write to
        * @param data New word value to write
        * @return Status of operation (true = success)
        */
        static bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
            return writeWords(devAddr, regAddr, 1, &data);
        }

        /** Write multiple bytes to an 8-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr First register address to write to
        * @param length Number of bytes to write
        * @param data Buffer to copy new data from
        * @return Status of operation (true = success)
        */
        static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
        	xpcc::I2cWriteReadAdapter adapter;

        	uint8_t tmp[length+1];
        	tmp[0] = regAddr;
        	memcpy(tmp+1, data, length);

        	adapter.initialize(devAddr, tmp, length+1, 0, 0);

        	while(!I2C_Master::startBlocking(&adapter));

        	return adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Idle;
        }


        /** Write multiple words to a 16-bit device register.
        * @param devAddr I2C slave device address
        * @param regAddr First register address to write to
        * @param length Number of words to write
        * @param data Buffer to copy new data from
        * @return Status of operation (true = success)
        */
        static bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {

        	xpcc::I2cWriteReadAdapter adapter;

        	uint8_t tmp[length*sizeof(uint16_t)+1];
        	tmp[0] = regAddr;
        	memcpy(tmp+1, data, length*sizeof(uint16_t));

        	adapter.initialize(devAddr, tmp, length*sizeof(uint16_t)+1, 0, 0);

        	while(!I2C_Master::startBlocking(&adapter));

        	return adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Idle;

        }

        static bool startRead(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
        	if(isBusy()) {
        		return false;
        	}

        	data[0] = regAddr;
        	adapter.initialize(devAddr, data, 1, data, length);

        	return I2C_Master::start(&adapter);
        }

        static bool isBusy() {
        	return adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy;
        }

        static xpcc::I2cWriteReadAdapter adapter;
        static uint16_t readTimeout;
};


#endif /* I2CDEV_HPP_ */

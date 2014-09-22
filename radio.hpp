/*
 * radio.hpp
 *
 *  Created on: Sep 18, 2014
 *      Author: walmis
 */

#ifndef RADIO_HPP_
#define RADIO_HPP_

#include <xpcc/architecture.hpp>
#include <RH_RF22.h>
#include <system.hpp>

using namespace xpcc;

class Radio : TickerTask, public RH_RF22 {
public:
	Radio() : RH_RF22(0, 1) {

	}

	void handleInit() {
		if(!init()) {
			panic("radio init fail");
		}
	}

    inline uint16_t getRxBad() {
    	return _rxBad;
    }

    inline uint16_t getRxGood() {
    	return _rxGood;
    }

    inline uint16_t getTxGood() {
    	return _txGood;
    }

    bool transmitting() {
    	return mode() == RHModeTx;
    }

    bool idle() {
    	return mode() == RHModeIdle;
    }

private:


    uint8_t spiBurstWrite0(uint8_t reg, const uint8_t* src, uint8_t len) {
        uint8_t status = 0;

        digitalWrite(_slaveSelectPin, LOW);
        status = radioSpiMaster::write(reg | RH_SPI_WRITE_MASK);

        while(len) {
        	uint8_t written = radioSpiMaster::burstWrite(src, len);
        	while(!radioSpiMaster::txFifoEmpty()) {
        		//TickerTask::yield();
        	}
        	len -= written;
        	src += written;
        }
        radioSpiMaster::flushRx();
        digitalWrite(_slaveSelectPin, HIGH);
        return status;
    }

    uint8_t spiBurstRead0(uint8_t reg, uint8_t* dest, uint8_t len) {
        uint8_t status = 0;

        digitalWrite(_slaveSelectPin, LOW);

        status = radioSpiMaster::write(reg & ~RH_SPI_WRITE_MASK); // Send the start address with the write mask off

        radioSpiMaster::flushRx();

        while(len) {
        	uint8_t n = radioSpiMaster::burstWrite(dest, len);
        	//wait until transfer finishes
        	while(radioSpiMaster::isBusy()) {
        		//TickerTask::yield();
        	}
        	radioSpiMaster::burstRead(dest, len);

        	len -= n;
        	dest += n;
        }
        digitalWrite(_slaveSelectPin, HIGH);

        return status;

    }
};


#endif /* RADIO_HPP_ */

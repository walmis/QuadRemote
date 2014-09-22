/*
 *
 *  Created on: May 7, 2014
 *      Author: walmis
 */
#ifndef EEPROM_HPP_
#define EEPROM_HPP_

#include <xpcc/architecture.hpp>
#include <xpcc/driver/storage/i2c_eeprom.hpp>

#define TOKEN 0x63

#define eeRead(elem, out) read<typeof(elem)>(offsetof(EEData, elem), out)
#define eeWrite(elem, out) write<typeof(elem)>(offsetof(EEData, elem), out); XPCC_LOG_DEBUG.printf("eew a:%d L:%d\n",offsetof(EEData, elem), sizeof(elem))


#include "eedata.hpp"

class Eeprom : public xpcc::I2cEeprom<xpcc::lpc17::I2cMaster1> {
public:
	Eeprom() : xpcc::I2cEeprom<xpcc::lpc17::I2cMaster1>(0x50), token(0) {}
	virtual ~Eeprom(){};

	void initialize()  {
		readByte(0, token);

		if(token != TOKEN) {
			write(0, (uint8_t*)&eeDefaults, sizeof(eeDefaults));
		}
	}

	bool isValidToken() {
		return token == TOKEN;
	}

	void setToken() {
		writeByte(0, TOKEN);
		token = TOKEN;
	}

protected:
	uint8_t token;
};

extern Eeprom eeprom;

#endif

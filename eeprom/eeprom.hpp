/*
 *
 *  Created on: May 7, 2014
 *      Author: walmis
 */
#ifndef EEPROM_HPP_
#define EEPROM_HPP_

#include <xpcc/architecture.hpp>
#include <xpcc/driver/storage/i2c_eeprom.hpp>

#include "eedata.hpp"

template<typename T, typename U> constexpr size_t offsetOf(U T::*member)
{
    return (char*)&((T*)nullptr->*member) - (char*)nullptr;
}

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

	template <typename T, typename U, typename Y>
	bool put(T U::*pos, Y &data) {
		static_assert(sizeof(T) == sizeof(Y), "Type size mismatch");
		return write(offsetOf(pos), (uint8_t*)&data, sizeof(T));
	}

	template <typename T, typename U, typename Y>
	bool get(T U::*pos, Y &dest) {
		static_assert(sizeof(T) <= sizeof(Y), "Type size mismatch");
		return read(offsetOf(pos), (uint8_t*)&dest, sizeof(T));
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

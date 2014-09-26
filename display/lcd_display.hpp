/*
 * LcdDisplay.hpp
 *
 *  Created on: Sep 9, 2014
 *      Author: walmis
 */

#ifndef LCDDISPLAY_HPP_
#define LCDDISPLAY_HPP_

#include <xpcc/processing.hpp>
#include <xpcc/io/stringstream.hpp>

class LCDLine {
public:
	LCDLine() {
		clear();
	}

	char line[16];
	uint8_t pos;

	void clear() {
		memset(line, ' ', sizeof(line));
		pos = 0;
	}

	template <typename T>
	void addField(T value, uint8_t fieldWidth = 0, bool align = true) {
		xpcc::StringStream<16> str;
		str << value;

		addField(str, fieldWidth, align);
	}
	template <typename T>
	LCDLine& operator <<(T val) {
		addField(val);
		return *this;
	}

private:

	void addField(xpcc::StringStream<16> &str, uint8_t fieldWidth, bool align) {
		if(align) {
			alignField(str, fieldWidth);
		}
		if(fieldWidth == 0) {
			fieldWidth = str.pos;
		}
		memcpy(line+pos, str.buffer, align?fieldWidth:str.pos);
		pos += fieldWidth;
	}

	void alignField(xpcc::StringStream<16> &str, uint8_t fieldWidth) {
		if(str.pos < fieldWidth) {
			uint8_t diff = fieldWidth - str.pos;

			memmove(str.buffer+diff, str.buffer, str.pos);
			memset(str.buffer, ' ', diff);
		}
	}


};
#include <xpcc/architecture.hpp>
template<typename T>
class LcdDisplay : xpcc::TickerTask {
public:
	LcdDisplay(T* lcd) : LCD(lcd) {
		page = 0;
	}

	void setPage(uint8_t page) {
		this->page = page;
	}

protected:

	void handleTick() {

		static xpcc::PeriodicTimer<> t(50);
		if(t.isExpired()) {
			line[0].clear();
			line[1].clear();

			getPage(page);

			LCD->setCursor(0, 0);
			LCD->xpcc::IOStream::write((uint8_t*)line[0].line, 16);
			LCD->xpcc::IOStream::write((uint8_t*)line[1].line, 16);
		}
	}

	virtual void getPage(uint8_t page) = 0;

	T* LCD;
	LCDLine line[2];
	uint8_t page;
};


#endif /* LCDDISPLAY_HPP_ */

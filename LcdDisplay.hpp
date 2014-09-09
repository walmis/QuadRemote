/*
 * LcdDisplay.hpp
 *
 *  Created on: Sep 9, 2014
 *      Author: walmis
 */

#ifndef LCDDISPLAY_HPP_
#define LCDDISPLAY_HPP_

#include <xpcc/processing.hpp>

using namespace xpcc;

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

	void addTextField(char *text, uint8_t fieldWidth = 0) {
		StringStream<16> str;

		str << text;

		addField(str, fieldWidth);
	}

	void addIntegerField(int i, uint8_t fieldWidth = 0) {
		StringStream<16> str;

		str << i;

		addField(str, fieldWidth);
	}

	void addFloatField(float n) {


	}

private:

	void addField(StringStream<16> &str, uint8_t fieldWidth) {
		alignField(str, fieldWidth);
		if(fieldWidth == 0) {
			fieldWidth = str.pos;
		}
		strncpy(line+pos, str.buffer, fieldWidth);
		pos += fieldWidth;
	}

	void alignField(StringStream<16> &str, uint8_t fieldWidth) {
		if(str.pos < fieldWidth) {
			uint8_t diff = fieldWidth - str.pos;

			memmove(str.buffer+diff, str.buffer, str.pos);
			memset(str.buffer, ' ', diff);
		}
	}


};

template<typename T>
class LcdDisplay : TickerTask {
public:
	LcdDisplay(T* lcd) : LCD(lcd) {
		page = 0;
	}

	void setPage(uint8_t page) {
		this->page = page;
	}

protected:

	void handleTick() {

		static PeriodicTimer<> t(50);
		if(t.isExpired()) {
			line[0].clear();
			line[1].clear();

			getPage(page);

			LCD->setCursor(0, 0);
			LCD->IOStream::write((uint8_t*)line[0].line, 16);
			LCD->IOStream::write((uint8_t*)line[1].line, 16);
		}
	}

	virtual void getPage(uint8_t page) = 0;

	T* LCD;
	LCDLine line[2];
	uint8_t page;
};


#endif /* LCDDISPLAY_HPP_ */

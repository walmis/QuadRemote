
#include <xpcc/architecture.hpp>

#include <wirish.h>
#include <RHGenericSPI.h>
#include <stdarg.h>
#include "pindefs.hpp"

int printf(const char* fmt, ...) {
	va_list ap;
	va_start(ap, fmt);

	XPCC_LOG_DEBUG .vprintf(fmt, ap);

	va_end(ap);

	return 0;
}

int puts(const char* s) {
	XPCC_LOG_DEBUG << s << xpcc::endl;
	return 1;
}

#ifdef putchar
#undef putchar
#endif

int putchar(int c) {
	XPCC_LOG_DEBUG << c;
	return 1;
}

void delay(uint32_t millis) {
	xpcc::delay_ms(millis);
}

void rh_yield() {
	//xpcc::TickerTask::yield();
}

void rh_atomic_block_start() {
	xpcc::GpioInt::disableInterrupts();
}

void rh_atomic_block_end() {
	xpcc::GpioInt::enableInterrupts();
}

uint32_t millis() {
	return xpcc::Clock::now().getTime();
}

void pinMode(uint8_t pin, WiringPinMode mode) {
	//printf("pinmode %d -> %d\n", pin, mode);

	if(pin == 0) {
		switch(mode) {
		case WiringPinMode::OUTPUT:
			radio_sel::setOutput();
		case WiringPinMode::INPUT:
			radio_sel::setInput();
		}
	}
}


void attachInterrupt(uint8_t pin, void (*fn)(void), int mode) {
	printf("attach int %d -> %d\n", pin, mode);

	if(pin == 1) {
		xpcc::GpioInt::attach(radio_irq::Port, radio_irq::Pin, fn, xpcc::IntEdge::FALLING_EDGE);
	}

}


void digitalWrite(uint8_t pin, uint8_t val) {
	if(pin == 0) {
		radio_sel::setOutput(val);
	}
}

class Spi : RHGenericSPI {

public:
	void begin() {}
	void end() {}

	uint8_t transfer(uint8_t data) {
		//printf("spi write %02x\n", data);
		return radioSpiMaster::write(data);
	}
};

Spi hardware_spi;

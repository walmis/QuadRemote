
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

void yield() {
	xpcc::TickerTask::yield();
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

void (*irqFn)() = 0;

void attachInterrupt(uint8_t pin, void (*fn)(void), int mode) {
	printf("attach int %d -> %d\n", pin, mode);

	if(pin == 1) {
		xpcc::GpioInterrupt::enableInterrupt(radio_irq::Port,
				radio_irq::Pin, xpcc::IntSense::EDGE,
				xpcc::IntEdge::SINGLE, xpcc::IntEvent::FALLING_EDGE);

		irqFn = fn;

		xpcc::GpioInterrupt::enableGlobalInterrupts();
	}

}

class Irq : xpcc::TickerTask {
	void handleInterrupt(int irq) {
		if(xpcc::GpioInterrupt::checkInterrupt(irq,
				radio_irq::Port, radio_irq::Pin, xpcc::IntEvent::FALLING_EDGE)) {

			if(irqFn)
				irqFn();

		}
	}
};

static Irq _irq;

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
		return xpcc::lpc17::SpiMaster1::write(data);
	}
};

Spi hardware_spi;

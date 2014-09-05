
#include <xpcc/architecture.hpp>

#include <wirish.h>
#include <RHGenericSPI.h>
#include <stdarg.h>

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

}

uint32_t millis() {
	return xpcc::Clock::now().getTime();
}

void pinMode(uint8_t pin, WiringPinMode mode) {
	printf("pinmode %d -> %d\n", pin, mode);
}

void attachInterrupt(uint8_t pin, void (*fn)(void), int mode) {
	printf("attach int %d -> %d\n", pin, mode);
}

void digitalWrite(uint8_t pin, uint8_t val) {
	printf("dig write %d -> %d\n", pin, val);
}

class Spi : RHGenericSPI {

public:
	void begin() {}
	void end() {}

	uint8_t transfer(uint8_t data) {
		printf("spi write %02x\n", data);
	}
};

Spi hardware_spi;

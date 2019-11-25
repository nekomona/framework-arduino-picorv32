/*
 *  PicoSoC - A simple example SoC using PicoRV32
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <Arduino.h>

#include "HardwareSerial_private.h"  // picorv32; work-a-round needed for Serial._rx_complete_irq();

// --------------------------------------------------------

extern uint32_t flashio_worker_begin;
extern uint32_t flashio_worker_end;

void flashio(uint8_t *data, int len, uint8_t wrencmd)
{
	((void(*)(uint8_t*, uint32_t, uint32_t))&flashio_worker_begin)(data, len, wrencmd);
}

void set_flash_qspi_flag()
{
	//set to winbond instruction set
	uint8_t buffer_rd1[2] = {0x05, 0};
	flashio(buffer_rd1, 2, 0);

	uint8_t buffer_rd2[2] = {0x35, 0};
	flashio(buffer_rd2, 2, 0);

	uint8_t buffer_wr[3] = {0x01, buffer_rd1[1], (uint8_t)(buffer_rd2[1] | 0x02)};
	flashio(buffer_wr, 3, 0x06);
}

void set_flash_mode_spi()
{
	reg_spictrl = (reg_spictrl & ~0x007f0000) | 0x00000000;
}

void set_flash_mode_dual()
{
	reg_spictrl = (reg_spictrl & ~0x007f0000) | 0x00400000;
}

void set_flash_mode_quad()
{
	reg_spictrl = (reg_spictrl & ~0x007f0000) | 0x00240000;
}

void set_flash_mode_qddr()
{
	reg_spictrl = (reg_spictrl & ~0x007f0000) | 0x00670000;
}

void enable_flash_crm()
{
	reg_spictrl |= 0x00100000;
}

// --------------------------------------------------------

void putchar(char c)
{
	if (c == '\n')
		putchar('\r');
	reg_uart_data = c;
}

void print(const char *p)
{
	while (*p)
		putchar(*(p++));
}

// --------------------------------------------------------

unsigned long millis()  // wiring.c
{
	uint32_t cycles_begin, cycles_now, cycles;
	__asm__ volatile ("rdcycle %0" : "=r"(cycles_begin));
	return cycles_begin/clk_div_ms;
}

unsigned long micros()  // wiring.c
{
	uint32_t cycles_begin, cycles_now, cycles;
	__asm__ volatile ("rdcycle %0" : "=r"(cycles_begin));
	return cycles_begin/clk_div_us;
}

void delay(unsigned long ms)  // wiring.c
{
	uint32_t start = micros();

	while (ms > 0) {
//		yield();  // picorv32: disabled for now (can be enabled/added if needed, see hooks.c)
		while ( ms > 0 && (micros() - start) >= 1000) {
			ms--;
			start += 1000;
		}
	}
}

void pinMode(uint8_t pin, uint8_t mode)  // wiring_digital.c
{
	/*
	if (pin < 8) {
		// gpio
		if (mode == INPUT) {
			reg_outp = (reg_outp & (~(0x00000001 << (pin+24))));  // set oe/dir bit[pin] to 0 (IOpin input)
		} else if (mode == OUTPUT) {
			reg_outp = (reg_outp | (0x00000001 << (pin+24)));     // set oe/dir bit[pin] to 1 (IOpin output)
#ifdef ASSERT_NOT_IMPLEMENTED
		} else {
			assert(true, "pinMode supports mode INPUT, OUTPUT only");  // not supported (yet)
#endif
		}
#ifdef ASSERT_NOT_IMPLEMENTED
	} else {
		assert(true, "pinMode supports pin 0..7 only");  // not supported (yet)
#endif
	}
	*/
}

void digitalWrite(uint8_t pin, uint8_t value)  // wiring_digital.c
{
	if (pin < 16) {
		if (value == LOW) {
			reg_outp = (reg_outp & (~(0x00000001 << pin)));
		} else {
			reg_outp |= (reg_outp | (0x00000001 << pin));
		}
#ifdef ASSERT_NOT_IMPLEMENTED
	} else {
		assert(true, "digitalWrite supports pin 0..15, LED_BUILTIN only");  // not supported (yet)
#endif
	}
}

int digitalRead(uint8_t pin)  // wiring_digital.c
{
	if (pin < 16) {
		return (reg_inp >> pin) & 0x01;
#ifdef ASSERT_NOT_IMPLEMENTED
	} else {
		assert(true, "digitalRead supports pin 0..15 only");  // not supported (yet)
#endif
	}
	return 0;
}

int analogRead(uint8_t pin)  // wiring_analog.c
{
	/*
	if (pin == A0) {
		return ((int)((uint8_t)((reg_inp & 0xFF000000) >> 24))) << 2;  // read adc0 value from highest 8 input bits (sampled @ 4 Hz)
#ifdef ASSERT_NOT_IMPLEMENTED
	} else {
		assert(true, "analogRead supports pin A0 only");  // not supported (yet)
#endif
	}
	*/
	return 0;
}

void analogWrite(uint8_t pin, int val)  // wiring_analog.c
{
	/*
#ifdef ASSERT_NOT_IMPLEMENTED
	assert(false, "analogWrite supports PWM0 only and ignores the pin variable");  // not supported (yet)
#endif
//	if (pin < 1) {
		reg_outp = (reg_outp & 0xFFFF00FF) | (((uint32_t)((uint8_t)val)) << 8);  // PWM0
//	}
	*/
}

#if defined(ASSERT_NOT_IMPLEMENTED)
void assert(const bool stop, const char *p)  // picorv32 work-a-round
{
	print("\nassert(): ");
	print(p);
	print("\n");
#ifdef HALT_ON_ASSERT
	if (stop) {
		print("execution stopped.\n");
		while (1);  // may be also flash led as error indicator?
	}
#endif
}
#endif

// the main function initializes the program
int main(void)
{
	// init

	// Set QSPI mode for fast execution
	set_flash_qspi_flag();
	set_flash_mode_quad();
	enable_flash_crm();
	
	setup();

	// the loop function runs over and over again forever
	while (1) {
		Serial._rx_complete_irq();  // picorv32; work-a-round as we do not have in interrupt (yet)
		loop();
//		if (serialEventRun) serialEventRun();
	}

//	return 0;
}

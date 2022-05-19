/*
 * Copyright (c) 2012 Fredrik Atmer, Bathroom Epiphanies Inc
 * http://bathroomepiphanies.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// CDC - Copyright (C) 2022 S.Fuchita (@soramimi_jp)

#include "usb.h"
#include <avr/interrupt.h>
#include <string.h>

#define CLOCK 16000000UL
#define SCALE 125
static unsigned short _scale = 0;
static volatile unsigned long _system_tick_count;
static volatile unsigned long _tick_count;
static volatile unsigned long _time_s;
static unsigned short _time_ms;
uint8_t interval_1ms_flag;
ISR(TIMER0_OVF_vect, ISR_NOBLOCK)
{
	_system_tick_count++;
	_scale += 16;
	if (_scale >= SCALE) {
		_scale -= SCALE;
		_tick_count++;
		_time_ms++;
		if (_time_ms >= 1000) {
			_time_ms = 0;
			_time_s++;
		}
		interval_1ms_flag = 1;
	}
}

extern "C" void led(char f)
{
	if (f) {
		PORTB |= 0x01;
	} else {
		PORTB &= ~0x01;
	}
}

static inline int clamp(int v, int min, int max)
{
	if (v > max) v = max;
	if (v < min) v = min;
	return v;
}

void setup()
{
	// 16 MHz clock
	CLKPR = 0x80; CLKPR = 0;
	// Disable JTAG
	MCUCR |= 0x80; MCUCR |= 0x80;

	PORTB = 0x00;
	PORTC = 0x00;
	DDRB = 0x01;
	DDRC = 0x04;
	TCCR0B = 0x02; // 1/8 prescaling
	TIMSK0 |= 1 << TOIE0;

	usb_init();
	while (!usb_configured()) {
		_delay_ms(100);
	}
}

void loop()
{
	char tmp[8];
	uint8_t n = usb_data_rx(tmp, sizeof(tmp));
	usb_data_tx(tmp, n);
}

int main()
{
	setup();
	sei();
	while (1) {
		loop();
	}
}


// USB-CDC ATMEGA32U2
// Copyright (C) 2022 S.Fuchita (@soramimi_jp)

#include "usb.h"
#include <avr/interrupt.h>
#include <string.h>

extern "C" uint8_t usb_read_available_();
extern "C" uint8_t usb_read_byte_();

uint8_t data_tx_buffer[TX_EP_SIZE];
uint8_t data_tx_buffer_i;
uint8_t data_tx_buffer_n;

uint8_t data_rx_buffer[256];
int data_rx_buffer_i;
int data_rx_buffer_n;

extern "C" void clear_buffers()
{
	data_tx_buffer_i = 0;
	data_tx_buffer_n = 0;
	data_rx_buffer_i = 0;
	data_rx_buffer_n = 0;
}

void usb_poll_tx()
{
	uint8_t tmp[TX_EP_SIZE];
	while (1) {
		int n = data_tx_buffer_n;
		n = n < sizeof(tmp) ? n : sizeof(tmp);
		if (n == 0) break;
		for (uint8_t i = 0; i < data_tx_buffer_n; i++) {
			tmp[i] = data_tx_buffer[data_tx_buffer_i];
			data_tx_buffer_i = (data_tx_buffer_i + 1) % sizeof(data_tx_buffer);
		}
		usb_data_tx(tmp, n);
		data_tx_buffer_n -= n;
	}
}


static inline void usb_poll_rx()
{
	int space = sizeof(data_rx_buffer) - 1 - data_rx_buffer_n;
	while (space > 0) {
		uint8_t tmp[16];
		int n = sizeof(tmp);
		n = usb_data_rx(tmp, n < space ? n : space);
		if (n == 0) break;
		for (uint8_t i = 0; i < n; i++) {
			int j = (data_rx_buffer_i + data_rx_buffer_n) % sizeof(data_rx_buffer);
			data_rx_buffer[j] = tmp[i];
			data_rx_buffer_n++;
		}
		space -= n;
	}
}

void usb_poll()
{
	usb_poll_tx();
	usb_poll_rx();
}

static inline int usb_read_available()
{
	return data_rx_buffer_n + usb_read_available_();
}

static inline uint8_t usb_read_byte()
{
	for (int i = 0; i < 2; i++) {
		if (data_rx_buffer_n > 0) {
			uint8_t c = data_rx_buffer[data_rx_buffer_i];
			data_rx_buffer_i = (data_rx_buffer_i + 1) % sizeof(data_rx_buffer);
			data_rx_buffer_n--;
			return c;
		}
		usb_poll_rx();
	}
	return 0;
}

void usb_write_byte(char c)
{
	while (1) {
		if (data_tx_buffer_n < sizeof(data_tx_buffer)) {
			int8_t i = (data_tx_buffer_i + data_tx_buffer_n) % sizeof(data_tx_buffer);
			data_tx_buffer[i] = c;
			data_tx_buffer_n++;
			if (data_tx_buffer_n >= TX_EP_SIZE) {
				usb_poll_tx();
			}
			return;
		}
		usb_poll();
	}
}

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
	while (!is_usb_configured()) {
		_delay_ms(100);
	}
}

void loop()
{
	usb_poll();
//	uint8_t tmp[8];
//	uint8_t n = usb_data_rx(tmp, sizeof(tmp));
//	usb_data_tx(tmp, n);
	if (usb_read_available()) {
		uint8_t c = usb_read_byte();
		usb_write_byte(c);
	}
}

int main()
{
	setup();
	sei();
	while (1) {
		loop();
	}
}


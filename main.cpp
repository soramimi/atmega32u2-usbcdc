// USB CDC-ACM example for ATMEGA32U4
// Copyright (C) 2022 S.Fuchita (@soramimi_jp)

#include "usb.h"
#include <avr/interrupt.h>
#include <string.h>

uint16_t led_setup_count = 0;
uint16_t led_pmode_count = 0;
uint16_t led_error_count = 0;

extern "C" uint8_t usb_read_available_();
extern "C" uint8_t usb_read_byte_();

uint8_t data_tx_buffer[TX_EP_SIZE];
uint8_t data_tx_buffer_i;
uint8_t data_tx_buffer_n;

uint8_t data_rx_buffer[32];
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
	for (uint8_t i = 0; i < data_tx_buffer_n; i++) {
		tmp[i] = data_tx_buffer[data_tx_buffer_i];
		data_tx_buffer_i = (data_tx_buffer_i + 1) % TX_EP_SIZE;
	}
	usb_data_tx(tmp, data_tx_buffer_n);
	data_tx_buffer_n = 0;
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

static inline int usb_write_available()
{
	return sizeof(data_tx_buffer) - data_tx_buffer_n;
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

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1

#define PIN_RESET 6
#define PIN_MOSI 2
#define PIN_MISO 3
#define PIN_SCK 1
#define PIN_LED1 0
#define PIN_LED2 5

static inline void pinMode(char pin, char value)
{
	if (pin == PIN_LED2) {
		if (value) { DDRD |= 1 << pin; } else { DDRD &= ~(1 << pin); }
	} else {
		if (value) { DDRB |= 1 << pin; } else { DDRB &= ~(1 << pin); }
	}
}

static inline void digitalWrite(char pin, char value)
{
	if (pin == PIN_LED2) {
		if (value) { PORTD |= 1 << pin; } else { PORTD &= ~(1 << pin); }
	} else {
		if (value) { PORTB |= 1 << pin; } else { PORTB &= ~(1 << pin); }
	}
}

static inline bool digitalRead(char pin)
{
	if (pin == PIN_LED2) {
		return (PIND >> pin) & 1;
	} else {
		return (PINB >> pin) & 1;
	}
}

static inline void led1(bool f)
{
	digitalWrite(PIN_LED1, f ? LOW : HIGH); // low active
}

static inline void led2(bool f)
{
	digitalWrite(PIN_LED2, f ? LOW : HIGH); // low active
}

#define SCALE 125
uint16_t _scale = 0;
ISR(TIMER0_OVF_vect, ISR_NOBLOCK)
{
	_scale += 16;
	if (_scale >= SCALE) {
		_scale -= SCALE;
		if (led_setup_count > 0) {
			led_setup_count--;
			bool f = (led_setup_count & 0xff) > 127;
			led1(f);
			led2(f);
		} else {
			if (led_pmode_count > 0) {
				led_pmode_count--;
				led1((led_pmode_count & 0xff) > 127);
			}
			if (led_error_count > 0) {
				led_error_count--;
				led2((led_error_count & 0xff) > 127);
			}
		}
	}
}

void setup()
{
	// 16 MHz clock
	CLKPR = 0x80;
	CLKPR = 0;
	// Disable JTAG
	MCUCR |= 0x80;
	MCUCR |= 0x80;

	PORTB = 0x00;
	PORTC = 0x00;
	DDRB = 0x01;
	DDRC = 0x04;

	TCCR0B = 0x02; // 1/8 prescaling
	TIMSK0 |= 1 << TOIE0;

	clear_buffers();

	usb_init();
	while (!is_usb_configured()) {
		_delay_ms(100);
	}
}

void loop()
{
	usb_poll();

	if (usb_read_available() > 0 && usb_write_available() > 0) {
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

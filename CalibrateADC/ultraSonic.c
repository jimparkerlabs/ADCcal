/*
 * ultraSonic.c
 *
 *  Created on: Jun 17, 2015
 *      Author: jimparker
 */
#include "ultraSonic.h"
#include <avr/io.h>
#include <util/delay.h>

#define SONIC_PIN  7
#define SONIC_PORT D

uint16_t readUltrasonic(void) {
//    DDR#SONIC_PORT |= (1 << #SONIC_PIN);
	DDRD |= (1 << SONIC_PIN);              // set to output
	PORTD &= ~(1 << SONIC_PIN);            // clear pin
	_delay_us(2);                          // wait a bit
	PORTD |= (1 << SONIC_PIN);             // send a 5 microsecond pulse
	_delay_us(10);                         //
	PORTD &= ~(1 << SONIC_PIN);            // clear pin
	DDRD &= ~(1 << SONIC_PIN);             // set to input
	while (~(PIND | ~(1 << SONIC_PIN)));   // wait while pin is clear

	uint16_t delay = 0;
	while (PIND & (1 << SONIC_PIN)) {      // while pin is set
		_delay_us(1);
		delay++;
	}
	return delay;
}

/* The measured distance from the range 0 to 400 cm */
uint16_t ultrasonicCentimeters(void) {
	return readUltrasonic()/4.08/2;  // 29 ???
}

/* The measured distance from the range 0 to 157 Inches */
uint8_t ultrasonicInches(void) {
	uint16_t sonicValue = readUltrasonic();
	return sonicValue/10.4/2;  // 74 ???
}


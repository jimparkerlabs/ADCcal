/*
 * ultraSonic.c
 *
 *  Created on: Jun 17, 2015
 *      Author: jimparker
 */
#include "ultraSonic.h"
#include <avr/io.h>
#include <util/delay.h>

#define SONIC_PORT B
#define SONIC_PIN PB0
#define SONIC_CONTROL_PORT D
#define SONIC_CONTROL_PIN PD7

uint16_t oldReadUltrasonic(void) {
//    DDR#SONIC_PORT |= (1 << #SONIC_PIN);
	DDRD |= (1 << SONIC_CONTROL_PIN);              // set control to output
	PORTD &= ~(1 << SONIC_CONTROL_PIN);            // clear control
	_delay_us(2);                          // wait a bit
	PORTD |= (1 << SONIC_CONTROL_PIN);             // send a 20 microsecond control pulse
	//_delay_us(20);                         //
	//PORTD &= ~(1 << SONIC_CONTROL_PIN);            // clear control

	DDRB &= ~(1 << SONIC_PIN);             // set to input
	printString("waiting for pulse ");
	while ((PINB & (1 << SONIC_PIN)) == 0);   // wait while pin is clear
	printString("pulse started! ");

	uint16_t delay = 0;
	while (PINB & (1 << SONIC_PIN)) {      // while pin is set
		_delay_us(1);
		delay++;
	}
	printString("pulse ended! ");
	PORTD &= ~(1 << SONIC_CONTROL_PIN);            // clear control
	return delay;
}

unsigned long inputCapturePulseWidth(void) {
	uint16_t start = 0, end = 0;
	uint8_t numOverflows1 = 0, numOverflows2 = 0;

	DDRB &= ~(1 << PB0);                   // set ICP1 to input
	//PORTB &= ~(1 << PB0);                  // turn off pull-up resistor
	PORTB |= (1 << PB0);                   // turn off pull-up resistor
	TCNT1 = 0x0000;                        // zero timer

	TCCR1B = 0x01;                         // clock select = divide-by-1; start timer
	//uint16_t test = 0;
	// LEADING EDGE DETECT
	TCCR1B |= (1 << ICES1);                // capture rising edge
	TIFR1 |= (1 << ICF1);                  // clear capture flag
	numOverflows1 = 0;
	printBinaryByte(PINB);
	//printString("\r\n");
	while (1) {
		//test++;
		if (TIFR1 & (1 << ICF1)) break;   // if captured leading edge, leave loop
		if (TIFR1 & (1 << TOV1)) {        // if overflow . . .
			numOverflows1++;              //   . . . count it
			TIFR1 |= (1 << TOV1);         //   . . . and clear it
		}
		if (numOverflows1 > 20) break;    // if we overflowed too often, we have waited too long
	}
	printString("(");
	printWord(ICR1);
	printString(" ");
	printBinaryByte(PINB);
	printString(")");

	if (TIFR1 & (1 << ICF1)) {           // if we captured a leading edge
		start = ICR1;                    // save time of capture

		// TRAILING EDGE DETECT
		TIFR1 |= (1 << ICF1);            // clear capture flag
		TCCR1B &= ~(1 << ICES1);         // start capturing falling edges
		numOverflows2 = 0;
		while (1) {
			if (TIFR1 & (1 << ICF1)) break; // if we capture a falling edge, leave loop
			if (TIFR1 & (1 << TOV1)) {      // if we overflow . . .
				numOverflows2++;            //   . . . count it
				TIFR1 |= (1 << TOV1);       //   . . . and clear it
			}
			if (numOverflows2 > 6) break;  // if we overflowed too often, the pulse is too long
		}

		if (TIFR1 & (1 << ICF1)) {       // if we captured the falling edge
			end = ICR1;                  // save it
			printWord(start);
			printString("--");
			printWord(end);
		}
		printString("|");
		printByte(numOverflows1);
		printString("--");
		printByte(numOverflows2);
		printString("= ");
	}
	unsigned long width = ((numOverflows2 * 0xffff) + end - start);
	return width;
}

unsigned long readUltrasonic(void) {
//    DDR#SONIC_PORT |= (1 << #SONIC_PIN);
	// set up timer 1;
	TIMSK1 = 0x00;
	TCCR1A = 0x00;
	TCCR1C = 0x00;

	// send 25 usec pulse on control pin
	DDRD |= (1 << SONIC_CONTROL_PIN);       // set to output
	PORTD &= ~(1 << SONIC_CONTROL_PIN);     // clear control pin
	_delay_us(5);                          	// wait 2 usec
	PORTD |= (1 << SONIC_CONTROL_PIN);      //
	_delay_us(1);                           // send a 10 microsecond pulse

	DDRB &= ~(1 << PB0);                    // set PB0 to input
	PORTB &= ~(1 << PB0);                   // turn off pull-up resistor

	uint8_t this = PINB;
	printBinaryByte(this);
	uint8_t last = this;
	uint16_t time = 0;
	while(1) {
		if (time >= 40) {
			this = PINB;
			if (last != this) {
				printBinaryByte(this);
				printString("  ");
				printWord(time);
				printString("  ");
				if (this & 0x01) printString("\r\n");
				time = 0;
			}
			last = this;
		}
		time++;
	}
	return inputCapturePulseWidth();        // read pulse width on input capture pin
}

/* The measured distance from the range 0 to 400 cm */
uint16_t ultrasonicCentimeters(void) {
	return readUltrasonic()/4.08/2;  // 29 ???
}

/* The measured distance from the range 0 to 157 Inches */
uint8_t ultrasonicInches(void) {
	uint16_t sonicValue = readUltrasonic();
	return sonicValue / 147; ///10.4/2;
}

uint8_t ultrasonicRangeToInches(uint16_t range) {
	return range / 147; ///10.4/2;
}


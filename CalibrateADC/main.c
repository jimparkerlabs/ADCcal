/*
 * main.c
 *
 *  Created on: Jun 17, 2015
 *      Author: jimparker
 */

#undef __AVR_ATmega16__
#include <avr/io.h>
#include <util/delay.h>
#include "pinDefines.h"
#include "USART.h"
#include "ultraSonic.h"


static inline void initADC0(void) {
	ADMUX |= (1 << REFS0);
	ADCSRA |= (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);
}

/*
void calibrateADC0(uint16_t *cal) {
	uint16_t lowADC, highADC;

	transmitByte(0x0d);  //  cr
	transmitByte(0x0a);  // lf
	printString("Set the ADC to its LOWEST setting and hit <enter>!");
	transmitByte(0x0d);  //  cr
	transmitByte(0x0a);  // lf

	while (receiveByte() != 0x0d) ;
	ADCSRA |= (1 << ADSC);  // start conversion
	loop_until_bit_is_clear(ADCSRA, ADSC);
	lowADC = ADC;
	printString(". . . Reading: ");
	printWord(lowADC);

	transmitByte(0x0d);  //  cr
	transmitByte(0x0a);  // lf
	printString("Set the ADC to its HIGHEST setting and hit <enter>!");
	transmitByte(0x0d);  //  cr
	transmitByte(0x0a);  // lf
	while (receiveByte() != 0x0d) ;
	ADCSRA |= (1 << ADSC);  // start conversion
	loop_until_bit_is_clear(ADCSRA, ADSC);
	highADC = ADC;
	printString(". . . Reading: ");
	printWord(highADC);

	if (lowADC > highADC) {
		printString("you got them backwards!  fixing it now...");
		highADC = lowADC;
		lowADC = ADC;
	}

	cal[0] = lowADC;
	cal[1] = highADC;

	return;
}
*/
uint8_t calibrateADC0(uint16_t *min, uint16_t *max) {
	uint16_t lowADC, highADC;

	printString("\n\rSet the ADC to its LOWEST setting and hit <enter>:");

	while (receiveByte() != 0x0d) ;
	ADCSRA |= (1 << ADSC);  // start conversion
	loop_until_bit_is_clear(ADCSRA, ADSC);
	lowADC = ADC;
	printString("\r\n. . . Reading: ");
	printWord(lowADC);

	printString("\n\rSet the ADC to its HIGHEST setting and hit <enter>:");
	while (receiveByte() != 0x0d) ;
	ADCSRA |= (1 << ADSC);  // start conversion
	loop_until_bit_is_clear(ADCSRA, ADSC);
	highADC = ADC;
	printString("\r\n. . . Reading: ");
	printWord(highADC);

	if (lowADC > highADC) {
		printString("\r\nyou got them backwards!  fixing it now...");
		highADC = lowADC;
		lowADC = ADC;
	}

	*min = lowADC;
	*max = highADC;

	uint16_t range = (highADC - lowADC);
	uint8_t rangeBits = 0;
	while((range >> rangeBits) > 0) rangeBits++;

	return (10 - rangeBits);
}

// button-handling routines
uint8_t buttonPressed(void) {
	static uint8_t buttonState;
	uint8_t pressed = 0;
	if ((PIND & (1 << PD6)) == 0) {
		_delay_ms(2);
		if ((PIND & (1 << PD6)) == 0)
			pressed=1;
	}
	buttonState = pressed;
	return pressed;
}

uint8_t newButtonPress(void) {
	static uint8_t buttonState;
	uint8_t pressed = 0;
	if ((PIND & (1 << PD6)) == 0) {
		_delay_ms(2);
		if ((PIND & (1 << PD6)) == 0)
			pressed=1;
	}
	uint8_t oldState = buttonState;
	buttonState = pressed;
	return ((pressed == 1) && (oldState == 0)); // pressed now, not before
}

int main(void) {
	uint8_t ledValue;
	uint16_t adcValue;
	uint16_t adcMin = 0, adcMax = 0x3ff;
	uint8_t adcMultShift = 0;  // default to no scaling;

	uint8_t i;

	initUSART();
	initADC0();
	LED_DDR = 0xff;
	DDRD &= ~(1 << PD6);    // set button port to input
	PORTD |= (1 << PD6);  // enable pull-up resistor

	printString("\r\nHello world!\r\r");

	adcMultShift = calibrateADC0(&adcMin, &adcMax);

	printString("\r\ncalibrate to [min=");
	printWord(adcMin);
	printString(", max=");
	printWord(adcMax);
	printString("] -> shift=");
	printByte(adcMultShift);
	printString("\r\n");

	while (1) {
		// read the ADC (currently a photoresistor
		ADCSRA |= (1 << ADSC);  // start conversion
		loop_until_bit_is_clear(ADCSRA, ADSC);
		adcValue = ADC;
		uint16_t cal = (adcValue - adcMin) << adcMultShift;

		// turn on the LEDS as a bar graph
		ledValue = (cal >> 7);

		LED_PORT = 0;
		for(i=0; i <= ledValue; i++) {
			LED_PORT |= (1 << i);
		}

		if (newButtonPress() || (getByte() == 'x')) {  // if you send an 'x' or hit the button
			//float cal = ((float)(adcValue - adcCal[0]) / (float)(adcCal[1] - adcCal[0])) * (float)0x03ff;
			adcValue =  cal;

			// log sensor reading to serial port
			printString("resistor reading: ");
			printWord(adcValue);
			printString(", calibrated: ");
			printWord(cal);
			/*
			printString(" (");
			printBinaryByte(cal);
			printString(") ");
			*/
			printString(" | ultrasonic: ");
			printWord(readUltrasonic());
			transmitByte(32);  //  sp
			printByte(ultrasonicInches());
			printString("\r\n");

			//_delay_ms(500);
		}
	}
	return 0;
}

/*
 * ultraSonic.h
 *
 *  Created on: Jun 17, 2015
 *      Author: jimparker
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_
#include <avr/io.h>

unsigned long readUltrasonic();  // read the pulse width in clock cycles (microseconds at 1 Mhz)
uint16_t ultrasonicCentimeters(void);  // The measured distance in the range 0 to 400 cm
uint8_t ultrasonicInches(void);  // The measured distance in the range 0 to 157 Inches
uint8_t ultrasonicRangeToInches(uint16_t range);  // The measured distance in the range 0 to 157 Inches

#endif /* ULTRASONIC_H_ */

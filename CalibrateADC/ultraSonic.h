/*
 * ultraSonic.h
 *
 *  Created on: Jun 17, 2015
 *      Author: jimparker
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_
#include <avr/io.h>

uint16_t readUltrasonic();  // read the distance from the ultrasonic peripheral
uint16_t ultrasonicCentimeters(void);  // The measured distance in the range 0 to 400 cm
uint8_t ultrasonicInches(void);  // The measured distance in the range 0 to 157 Inches

#endif /* ULTRASONIC_H_ */

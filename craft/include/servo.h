/* Name: servo.h
 * Author: Alex Feinland
 * Description: Header file for servo.c.
 */


#ifndef _SERVO_H_
#define _SERVO_H_

#include <avr/io.h>

/* The servo's signal wire needs to be connected to a PWM toggled pin on the uC.
 * PD5 (OC1A) on the ATMega1284p will be used for this purpose. TCNT1 must be used to toggle OC1A.
 */
#define     SERVO_OUTPUT_PIN        PD5     // OC1A is PD5 on the ATMega1284p.
#define     SERVO_CONTROL_DDRx      DDRD    // The pin must be set as output in order to be toggled.


extern void servo_init(void);


#endif /* _SERVO_H_ */

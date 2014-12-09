/* Name: 28BYJ-48.h
 * Author: Alex Feinland
 * Description: Header file for 28BYJ-48.c
 */

#ifndef _28BYJ_48_H_
#define _28BYJ_48_H_


#include <avr/io.h>


/*
    The 28BYJ-48 has eight unique phases, output on the first four bits of PORTA.    
*/
#define     STEPPER_MOTOR_CONTROL_PORTx     PORTA
#define     STEPPER_MOTOR_CONTROL_DDRx      DDRA
#define     STEP_A      0x01
#define     STEP_AB     0x03
#define     STEP_B      0x02
#define     STEP_BC     0x06
#define     STEP_C      0x04
#define     STEP_CD     0x0C
#define     STEP_D      0x08
#define     STEP_DA     0x09



/* The sequence of steps for the motor. */
extern const uint8_t steps[8];


void stepper_motor_init(void);

/*
 * This funciton moves the motor one step per call.
 */
void step_motor(void);





#endif /* _28BYJ_48_H_ */

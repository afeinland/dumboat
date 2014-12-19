/* Name: 28BYJ-48.c
 * Author: Alex Feinland
 * Description: This file contains the necessary code to operate
 * the 28BYJ-48 stepper motor on the ATmega1284p.
 */

#include <avr/io.h>
#include "28BYJ-48.h"

/* This variable is checked in ISR(TIMER3_COMPA_vect) to determine
 * whether the motor should be moving or not. */

/* The sequence of steps for the motor. */
const uint8_t steps[8] = {STEP_A, STEP_AB, STEP_B, STEP_BC, STEP_C, STEP_CD, STEP_D};

void stepper_motor_init(void)
{
    // Set the first four bits on the port as output.
    STEPPER_MOTOR_CONTROL_DDRx = 0x0F;
}


/*
 * This funciton moves the motor one step per call.
 */
void step_motor(void)
{
    static uint8_t step_index = 0;

    if(step_index >= 8)
    {
        step_index = 0;
    }

    STEPPER_MOTOR_CONTROL_PORTx = steps[step_index];

    step_index++;
}


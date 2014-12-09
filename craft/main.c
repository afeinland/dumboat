/* Name: main.c
 * Author: Alex Feinland
 * Description: This main function is run on the ATMega1284p located on the craft.
 */

#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer.h"
#include "servo.h"
#include "28BYJ-48.h"



ISR(TIMER3_COMPA_vect)
{
    step_motor();

    /* TODO:
     *
     * if(XBee tells us to turn the motor)
     * {
     *   step_motor();
     * }
     */
}


int main(void)
{
    stepper_motor_init();
    servo_init();

    timers_on();



    while(1)
    {
        // Servo position 90 deg. to back of boat (rudder straight).
        OCR1A = 180;

    /* TODO:
     *
     * (1) rx stepper_motor value (via XBee) which tells us wether
     * we should be turning the motor or not.
     *
     * (2) rx servo value telling us what position the servo should be in.
     * ** What should the 'units' of this value be?
     */


    }

    return 0;   /* never reached */
}


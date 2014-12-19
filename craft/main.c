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
    if(stepper_motor_state == ON)
    {
        step_motor();
    }
}

void process_message(uint8_t msg)
{
    /* Check whether the message was for the stepper_motor.
     * If not, then the message is for the servo. */
    if(STEPPER_MOTOR_ON == msg)
    {
        stepper_motor_state = ON;
    }
    else if(STEPPER_MOTOR_OFF == msg)
    {
        stepper_motor_state = OFF;
    }
    else
    {
        /* The message wasn't for the stepper_motor, so it was for the servo. */
        OCR1A = msg;
    }
}

int main(void)
{
    stepper_motor_init();
    servo_init();
    timers_on();
    USART0_init();

    DDRB |= 0x01; // Set PB0 to output.

    uint8_t servo_position = 0;
    uint8_t rx_data = 0;

    while(1)
    {
        /* Read mesage sent from controller. */
        rx_data = USART0_rx_byte();
        process_message(rx_data);

    }

    return 0;   /* never reached */
}


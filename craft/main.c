/* Name: main.c
 * Author: Alex Feinland
 * Description: This main function is run on the ATMega1284p located on the craft.
 */

#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer.h"
#include "servo.h"
#include "28BYJ-48.h"

#define     F_CPU                       8000000UL // Assume CPU runs at 8mhz
#define     BAUD_RATE                   9600
#define     BAUD_RATE_REGISTER_VALUE    ((F_CPU / ( 16UL * BAUD_RATE)) - 1)

/* USART0_init(void)
 * Initializes the USART functionality on the ATMega328.
 */
void USART0_init(void)
{
    /* Set baud rate. TODO better comment here */
    UBRR0L = BAUD_RATE_REGISTER_VALUE;
    UBRR0H = BAUD_RATE_REGISTER_VALUE >> 8;

    /* ENable transmission and receive functionality.
     * On the ATMega328, the RXD and TXD are pins PD0 and PD1, respectively.
     */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    /* UCSZ00 and UCSZ01 set the USART Character SiZe to 8 bits.
     * That is, there are 8 bits of data in each TX/RX frame.
     */
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);

}

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

    USART0_init();

    timers_on();


    DDRB |= 0x01; // Set PB0 to output.

    while(1)
    {

    /* TODO:
     *
     * (1) rx stepper_motor value (via XBee) which tells us wether
     * we should be turning the motor or not.
     *
     * (2) rx servo value telling us what position the servo should be in.
     * ** What should the 'units' of this value be?
     */


        /******** Testing USART functionality between craft and controller uCs. ********/

        // read from USART.
        while( !(UCSR0A & (1 << RXC0)));
        if(UDR0 == 0xAA)
        {
            _delay_ms(100);
            PORTB ^= 0x01;
        }
    }

    return 0;   /* never reached */
}


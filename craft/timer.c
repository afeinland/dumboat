/* Name: timer.c
 * Author: Alex Feinland
 * Description: Timer setup for the servo and stepper motor on the ATMega1284p.
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer.h"


/* This function configures TCNT3 for Mode 4 with a 2ms tick. In Mode 4, TCNT3 counts
 * from BOTTOM (which is 0 in Mode 4) to TOP (the value held by OCR3A, in Mode 4), incrementing by one each CPU cycle.
 */
void timer_init_stepper_motor(void)
{
    /* Clear timer/counter 3. */
    TCNT3 = 0;

    /* Sets the I-Flag in the status register, enabling global interrupts. */
    SREG |= 0x80;

    /* OCR3A holds the TOP value for TCNT3 in Mode 4. With a CPU frequency of 8mhz, prescale value
     * of 64 (configured below), and desired tick frequency of 2ms, we get a TOP value of 250.
     * This is because every 250 CPU ticks, 2ms in absolute time till pass.
     */
    OCR3A = 250;

    /* The OCIE3A bit in TIMSK3 enables the Timer/Counter3 Output Compare A Match interrupt.
     * This interrupt sets the OCF3A flag when TCNT3 = OCR3A, thus executing the TIMER3_COMPA_vect ISR.
     * The I-flag in SREG must also be set to enable this interrupt.
     */
    TIMSK3 = (1 << OCIE3A);

    /* WGM32 set the timer to Mode 4.
     *
     * CS31 and CS30 set the prescale value to 64.
     */
    TCCR3B = (1 << WGM32) | (1 << CS31) | (1 << CS30);
}


/* This function configures TCNT1 for Mode 14 with a PWM period of 50hz.
 */
void timer_init_servo(void)
{
    /* Clear timer/counter 1. */
    TCNT1 = 0;

    /* ICR1 is the TOP value for TCNT1 in Mode 14. TCNT1 increments once each clock tick,
     * or, with CPU a frequency of 8mhz and prescale value of 64, every 8us. We want TCNT1 to
     * reach TOP every 20ms (50hz), so the value for TOP can be determined by 20ms/8us = 2500.
     * That is, for every 2500 8us periods, 20ms (one PWM period) will have elapsed.
     *
     * The TOP value can be calculated in one step using the formula:
     * TOP = CPUFreq/(PrescaleValue * PWMFreq) - 1. Subtract one because we're counting from 2499 to 0.
     *
     * Simply put, ICR1 determines the period for the timer.
     */
    ICR1 = 2499;
    
    /* COM1A1 and COM1B1 allow toggling of the OC1A and OC1B pins, repsectively.
     * The DDRx register needs to be set appropriately to enable the output, however.
     *
     * WGM11, WGM12, and WGM13 enable Mode 14.
     *
     * CS11 and CS10 set the prescale value to 64.
     */
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); 
    TCCR1B = (1 << WGM13)  | (1 << WGM12)  | (1 << CS11) | (1 << CS10);
}


void timers_on(void)
{
    timer_init_servo();
    timer_init_stepper_motor();
}



/* Name: main.c
 * Author: Alex Feinland
 * Description: This is the main function for the controller..
 */


/* Current state of this file:
 *
 * The joystick is connected to ADC0 and ADC1. The ATMega328 reads ADC1 every 2ms (for the motor)
 * and reads ADC0 constantly (for the servo). ADC0 is output on eight LEDs connected to PORTD.
 * There is a lot of noise on the LEDs, perhaps because the sample rate of ADC0 is too great?
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>



/* Either of these values are written to bits 0:3 of the ADMUX
 * register when reading from the ADC.
 */
#define     ADC_CHANNEL_SERVO              0x00
#define     ADC_CHANNEL_STEPPER_MOTOR      0x01

/* The up/down (UD) channel on the joystick controls the servo, which in turn steers the boat.
 * The left/right (LR) channel controls the motor, which makes the boat go forward.
 * The reason UD make the boat go left or right and LR makes the boat go forwards (or "up") is
 * because the joystick is rotated 90 degrees clockwise in order to fit on my breadboard.
 *
 * I.e., moving the joystick "up" is equivalent to moving it to the left if it were oriented properly.
 */
#define     JOYSTICK_LR_EQUILIBRIUM_VALUE   129
#define     JOYSTICK_UD_EQUILIBRIUM_VALUE   127



void timers_on(void)
{
    /* Here we set up Timer/Counter1 in Mode 14. 
     */
    TCNT1  = 0; // Clear the timer.
    
    /* This is the TOP value for T/C1 in mode 14.u */
    ICR1   = 2499;
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12) | (1 << CS11) | (1 << CS10);
}



void ADC_init(void)
{
    ADMUX  |= (1 << ADLAR);
    DIDR0   = 0x3F;
    ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADPS2) | (1 << ADPS1);
}

/*
 * DESCRIPTION: Read the value on ADC pins 0:5 (pins 23:28 on the ATMega328).
 *
 * INPUTS:
 *      PARAMETERS:
 *          uint8_t     ADCx        The ADC pin to be read from. Valid values are 0:5.
 *
 * OUTPUTS:
 *      RETURN:
 *          uint8_t     ADCH        The value in the (8-bit) ADC High register.
 *
 * NOTES:   Since we set the ADLAR bit in the ADMUX register (in ADC_init()), ADCH contains
 *          the highest 8 bits from the output of the ADC. 8 bits provide enough resolution
 *          for our needs, so we only need to return the value from the ADCH register
 *          and not the entire ADC register (both ADCH and ADCL).
 */
uint8_t ADC_read(uint8_t ADCx)
{
    // Clear the four channel selection bits (0:3), but save the ADLAR bit.
    ADMUX &= 0xF0;

    // Valid values for ADCx are 0:5. If an invalid value is given, ADC0 is read from.
    ADMUX |= (ADCx <= 0x05) ? ADCx : 0x00;

    // Wait for channel to stabilize.
    static uint8_t i = 0;
    for(i = 0; i < 15; ++i)
    {
        asm("nop");
    }

    return ADCH;
}

/*
 * DESCRIPTION: Arbitrary mapping for joystick position to servo position.
 *              Min/max values from joystick are 0/255, respectively (8 bit precision
 *              because we're only reading from left aligned ADCH). Arbitrary min/max
 *              values for servo are 120/240, respectively. 
 *
 * INPUTS:
 *      PARAMETERS:
 *          uint8_t     joystick_value      The 8-bit value output from the ADC.
 *
 * OUTPUTS:
 *      RETURN:
 *          uint8_t     servo_value         The number of clock cycles that the PWM shall remain high.
 *
 * NOTES:   The possible values for servo_value are dependent on the CPU clock frequency.
 *          Changing the CPU clock frequency will probably render these values meaningless.
 */
uint8_t joystick_to_servo(uint8_t joystick_value)
{
    uint8_t servo_value = 180; // "Neutral" 90 degree position.

    // TODO These magic numbers should probably be #defines, or at least have some explanation.
    if(joystick_value > 218)
    {
        servo_value = 240;
    }
    else if(joystick_value > 180)
    {
        servo_value = 220;
    }
    else if(joystick_value > 130)
    {
        servo_value = 200;
    }
    else if(joystick_value > 123)
    {
        servo_value = 180;
    }
    else if(joystick_value > 78)
    {
        servo_value = 160;
    }
    else if(joystick_value > 36)
    {
        servo_value = 140;
    }
    else
    {
        servo_value = 120;
    }

    return servo_value;
}


int main(void)
{
    ADC_init();
    timers_on();

    DDRD = 0xFF; // For ADC outpt to LEDs.

    uint8_t tmpADC;

    while(1)
    {
        tmpADC = ADC_read(ADC_CHANNEL_STEPPER_MOTOR);
        PORTD  = tmpADC;
        tmpADC = ADC_read(ADC_CHANNEL_SERVO);

    }

    return 0;   /* never reached */
}


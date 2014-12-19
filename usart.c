/* Name: usart.c
 * Author: Alex Feinland
 * Description: This file contains the functions necessary for communicating to the XBee radios via USART.
 */


#include "usart.h"


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


/* USART0_read_byte(void)
 * Reads one byte from USART0 (pin RXD0).
 * On the ATMega328, there is only one RXD port located on pin 30.
 * On the ATMega1284p, the RXD0 port is located on pin 14. 
 */
uint8_t USART0_rx_byte(void)
{
    // Wait for data rx (one byte).
    while( !(UCSR0A & (1 << RXC0)))
        ;

    return UDR0;
}


void USART0_tx_byte(uint8_t byte)
{
    /* Wait for the TX buffer to empty.
     * The USART Data Register Empty bit (UDRE0) in UCSR0A
     * is set when the tx buffer is empty. */
    while( !(UCSR0A & (1 << UDRE0)))
        ;

    UDR0 = byte;
}

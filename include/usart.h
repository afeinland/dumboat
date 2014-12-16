/* Name: usart.h
 * Author: Alex Feinland
 * Description: This is the header file for usart.c.
 * There is only one USART (USART0) on the ATMega328. The ATMega1284p has two USART,
 * one of which is USART0. So, conveniently, both uC's can use these functions.
 */


#ifndef _USART_H_
#define _USART_H_

#include <avr/io.h>


#define     F_CPU                       8000000UL // Assume CPU runs at 8mhz
#define     BAUD_RATE                   9600
#define     BAUD_RATE_REGISTER_VALUE    ((F_CPU / ( 16UL * BAUD_RATE)) - 1)



/* USART0_init(void)
 * Initializes the USART functionality on the ATMega328 and the ATMega1284p.
 */
void USART0_init(void);


/* USART0_read_byte(void)
 * Reads one byte from USART0 (pin RXD0).
 * On the ATMega328, there is only one RXD port located on pin 30.
 * On the ATMega1284p, the RXD0 port is located on pin 14. 
 */
uint8_t USART0_rx_byte(void);


void USART0_tx_byte(uint8_t byte);

#endif /* _USART_H_ */

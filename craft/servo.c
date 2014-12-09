/* Name: servo.c
 * Author: Alex Feinland
 * Description: This file contains the necessary code to operate
 * some random servo I found lying around my room on the ATmega1284p.
 */

#include "servo.h"

void servo_init(void)
{
   SERVO_CONTROL_DDRx = (1 << SERVO_OUTPUT_PIN);
}



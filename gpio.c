/* This is the c file for the general purpose input output inlcude file
 * Created: May 17, 2018 by Jan van Deventer
 * Modified: May 17, 2018 Jan van Deventer
 */

#include <avr/io.h> // input output header file for this AVR chip.
#include "gpio.h"

int initGPIO(void)
{
	//Set up input output direction on Port B, C and G
	DDRB |= (1<<DISPLAY_LED);	// Set the display back light's IO pin an an output. Leave other bits as they were.
	DDRC = 0b00000111;		// Set the direction of the IO pins on Port C to output on the 3 least significant bits and input on the 5 higher ones. 5 buttons and 3 LEDs.
	DDRG |= 0b00000011;		// set the direction of the IO pins on Port G's lower 2 bytes as output (LEDs 1 & 2). Leave the other bits as they were.
	return(0);
}

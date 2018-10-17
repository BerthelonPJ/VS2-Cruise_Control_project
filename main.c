//
//  main.c
//  Cruise Control
//
//  Created by Pierre-Jean Berthelon on 04/10/2018.
//  Copyright © 2018 Pierre-Jean Berthelon. All rights reserved.
//

#include <stdio.h>
#include <string.h>
#include <avr/io.h> // input output header file for this AVR chip.
#include <avr/pgmspace.h>    // Contains some type definitions and functions.
#include <avr/interrupt.h>    // Contains ISR (Interrupt Service Routines) or interrupt handler details
#include "global.h"
#include "lcd.h"
#include "gpio.h"
#include "usart.h"
#include "spi.h"

#define DISPLAYLENGTH 16
#define DTOP DSTEER

<<<<<<< HEAD
#define LM77_ADDR	0x48 // I2C address of the LM77 temperature sensor

enum dStates {DBOOT,DSPEED,DTEMP,DACCEL,DSTEER};    /* enumeration of states (C programming, p) */
char *dbStateName[] = {"Coucou ça boot","Speed", "Temp.", "Accel.", "Steer"}; /* initialization of Pointer Array*/
=======
enum dStates {DBOOT,DSPEED,DTEMP,DACCEL,DSTEER};    /* enumeration of states (C programming, p) */
char *dbStateName[] = {"Coucou ça boot","Speed", "Steering Angle", "Temp.", "Accel."}; /* initialization of Pointer Array*/
>>>>>>> 225cecc5650445554b23961d337505651e3188fd
volatile unsigned int dbState;        /* display's state (activated by buttons)*/
volatile unsigned char buttons;        // This registers holds a copy of PINC when an external interrupt 6 has occurred.
volatile unsigned char bToggle = 0;    // This registers is a boolean that is set when an interrupt 6 occurs and cleared when serviced in the code.
volatile unsigned char textEdit= 0;    // Boolean to enable text editing.
volatile uint16_t adc_value;  //Allocate the double byte memory space into which the result of the 10 bits Analog to Digital Converter (ADC) is stored.
volatile int pWidth; // Pulse width for the servo motor on Timer Counter 3

int initADC(){
    //Set up analog to digital conversion (ADC)
    //ADMUX register
    //AVcc with external capacitor on AREF pin (the 2 following lines)
    ADMUX &= ~(1<<REFS1);  //Clear REFS1 (although it should be 0 at reset)
    ADMUX |= (1<<REFS0);   //Set REFS0
    ADMUX &= (0b11100000); //Single ended input on ADC0
    ADMUX &= ~(1<<ADLAR);  //Making sure ADLAR is zero (somehow it was set to 1)
    //The ACDC control and status register B ADCSRB
    ADCSRB &= ~(1<<ADTS2) & ~(1<<ADTS1) & ~(1<<ADTS0);  //Free running mode
    //The ADC control and status register A ADCSRA
    ADCSRA |= (1<<ADPS2) | (1<<ADPS1) |(1<<ADPS0);//set sampling frequency pre-scaler to a division by 128
    ADCSRA |= (1<<ADEN)  | (1<<ADATE) | (1<<ADIE);//enable ADC, able ADC auto trigger, enable ADC interrupt
    return(0);
}

/** This function sets up the External Interrupt, and specifically Interrupt 6 that is connected to the push buttons */
int initExtInt(void)
{
    //Set up external Interrupts
    // The five Switches are ORed to Pin PE6 which is alternatively Int6
    DDRC = 0b00000111;		// Set the direction of the IO pins on Port C to output on the 3 least significant bits and input on the 5 higher ones. 5 buttons and 3 LEDs.
    DDRG |= 0b00000011;
    EICRB |= (0<<ISC61) | (1<<ISC60);  //Any logical change to INT6 generates an interrupt
    EIMSK |= (1<<INTF6);
    return(0);
}

/** This function initializes the I2C clock */
void setupTWI()
{
	//Setting up TWI baud rate to 100kHz
	TWBR = 72;		//Look at formula on page 210;
	TWSR &= ~(1<<TWPS1) & ~(1<<TWPS0); //With no pre-scaler
}


/** This function initializes the LCD display and should be called only once before the while(1) loop in the main(). */
int initDisplay(void)
{
    dbState = DBOOT;
    lcdInit();    //initialize the LCD
    lcdClear();    //clear the LCD
    lcdHome();    //go to the home of the LCD
    lcdPrintData(dbStateName[dbState], strlen(dbStateName[dbState])); //Display the text on the LCD
    PORTB |= 1 << DISPLAY_LED;    // Turn on the display's back light.
    return(0);
}


/** This function is called when cycling up the states.*/
int dbStateUp(void)
{
    if (++dbState > DTOP)
        dbState = DSPEED;
    lcdClear();    //clear the LCD
    lcdHome();    //go to the home of the LCD
    lcdPrintData(dbStateName[dbState], strlen(dbStateName[dbState])); //Display the text on the LCD
    return 0;
}

/** This function is called when cycling down the states.*/
int dbStateDown(void)
{
    if (dbState-- <= DSPEED)
        dbState = DTOP;
    lcdClear();    //clear the LCD
    lcdHome();    //go to the home of the LCD
    lcdPrintData(dbStateName[dbState], strlen(dbStateName[dbState])); //Display the text on the LCD
    return 0;
}
/*int brake(void)
{
	return 0;
}*/
// The purpose of this function is to get the current_speed of the car.
/** This function uses the push buttons to let the user to change states upon boot up. It is also used to enter in Coffee maker mode*/
int DbBOOThandler(void)
{
    switch(buttons & 0b11111000){
        case 0b10000000:            //S5 center button
            /* do nothing */
            break;
        case 0b01000000:            //S4  upper button
            dbStateUp();
            break;
        case 0b00100000:            //S3 left button
            /* do nothing */
            break;
        case 0b00010000:            //S2 lower button
            dbStateDown();
            break;
        case 0b00001000:            //S1 right button
            /* do nothing */
            break;
        default:
            /* button released, do nothing */
            break;
    }
    return 0;
}
/** This function uses the push buttons to let the user to change states upon boot up. It is also used to enter in Coffee maker mode*/
int DbSPEEDhandler(void)
{
    switch(buttons & 0b11111000){
        case 0b10000000:            //S5 center button
            //brake();
            break;
        case 0b01000000:            //S4  upper button
            dbStateUp();
            break;
        case 0b00100000:            //S3 left button
            /* change channel using ADMUX */
            break;
        case 0b00010000:            //S2 lower button
            dbStateDown();
            break;
        case 0b00001000:            //S1 right button
            /* change channel using ADMUX */
            break;
        default:
            /* do nothing */
            break;
    }
    return 0;
}
/** This function uses the push buttons to let the user to change states upon boot up. It is also used to enter in Cleaning mode*/
int DbSTEERhandler(void)
{
    switch(buttons & 0b11111000){
        case 0b10000000:            //S5 center button
            //brake();
            break;
        case 0b01000000:            //S4  upper button
            dbStateUp();
            break;
        case 0b00100000:            //S3 left button
            /* change channel using ADMUX */
            break;
        case 0b00010000:            //S2 lower button
            dbStateDown();
            break;
        case 0b00001000:            //S1 right button

            /* change channel using ADMUX */
            break;
        default:
            /* do nothing */
            break;
    }
    return 0;
}
/** This function uses the push buttons to let the user to change states upon boot up. It is also used to enter in boiling water mode*/
int DbTEMPhandler(void)
{
    switch(buttons & 0b11111000){
        case 0b10000000:            //S5 center button
            //brake();
            break;
        case 0b01000000:            //S4  upper button
            dbStateUp();
            break;
        case 0b00100000:            //S3 left button
            /* change channel using ADMUX */
            break;
        case 0b00010000:            //S2 lower button
            dbStateDown();
            break;
        case 0b00001000:            //S1 right button
            /* change channel using ADMUX */
            break;
        default:
            /* do nothing */
            break;
    }
    return 0;
}
/** This function uses the push buttons to let the user to change states upon boot up. It is also used to enter in boiling water mode*/
int DbACCELhandler(void)
{
    switch(buttons & 0b11111000){
        case 0b10000000:            //S5 center button
            //brake();
            break;
        case 0b01000000:            //S4  upper button
            dbStateUp();
            break;
        case 0b00100000:            //S3 left button
            /* change channel using ADMUX */
            break;
        case 0b00010000:            //S2 lower button
            dbStateDown();
            break;
        case 0b00001000:            //S1 right button
            /* change channel using ADMUX */
            break;
        default:
            /* do nothing */
            break;
    }
    return 0;
}
void error(unsigned char val)
{
	unsigned char text[16];
	sprintf(text,"Error: 0x%02X",val);
	lcdGotoXY(0,1);
	lcdPrintData(text, strlen(text)); //Display the text on the LCD
}


int main(void) {
<<<<<<< HEAD
  //unsigned char temp ;		//Allocate memory for  temp
  char temp, tempLow ;		//Allocate memory for  temp and temp2
=======
  unsigned char temp ;		//Allocate memory for  temp
>>>>>>> 225cecc5650445554b23961d337505651e3188fd
  char cursor = 0;				/* allocate a variable to keep track of the cursor position and initialize it to 0 */
  char textLine[DISPLAYLENGTH + 1];	/* allocate a consecutive array of 16 characters where your text will be stored with an end of string */
  char text[10];				//Allocate an array of 10 bytes to store text
  uint16_t adcBuffer;			// Allocate the memory to hold ADC results that is not disturbed by interrupts


  char debug;				//debug byte to look at the TWI status register code
  int temperature;		// Allocate a word for the temperature
  float curr_temp = 0;		// Floating type memory allocation for the temperature


  textLine[0] = 'A';				/* initialize the first ASCII character to A or 0x41 */
  textLine[1] = '\0';				/* initialize the second character to be an end of text string */
  temp = initGPIO();		//Set up the data direction register for both ports B, C and G
  temp = initDisplay();	//Set up the display
  temp = initExtInt();	//Set up the external interrupt for the push buttons
  temp = initADC();		// Setup the Analog to Digital Converter
  //TimerCounter0setup(128);// enable the dimming of the display backlight with PWM using TimerCounter 0 and pin OCR0
<<<<<<< HEAD
  setupTWI(); //Initialization of the TWI, for the temp sensor.
=======

>>>>>>> 225cecc5650445554b23961d337505651e3188fd
  ADCSRA |= (1<<ADSC);	//Start ADC
  sei();					// Set Global Interrupts

  while(1)
	{
		ADCSRA &= ~(1<<ADIE);		//disable ADC interrupt to prevent value update during the conversion
		adcBuffer = adc_value;
		ADCSRA |= (1<<ADIE);		//re-enable ADC interrupt

    OCR0A = adcBuffer >> 2;		// using the top 8 bits of the ADC, load OCR0A to compare to the timer Counter 0 to generate aPWM for the display back light
		if (adcBuffer>852)
    {			// turning on the LEDs as a function of the ADC, regardless of the state the software is in.
			PORTC = 0b00000111;
			PORTG |= 0b00000011;
		}
		else if(adcBuffer>682)
    {
			PORTC = 0b00000011;
			PORTG |= 0b00000011;
		}
		else if(adcBuffer>511)
    {
			PORTC = 0b00000001;
			PORTG |= 0b00000011;
		}
		else if(adcBuffer>341)
    {
			PORTC = 0b00000000;
			PORTG |= 0b00000011;
		}
		else if(adcBuffer>170)
    {
			PORTC = 0b00000000;
			PORTG |= 0b00000001;
			PORTG &= 0b11111101;
		}
		else
    {
			PORTC = 0b00000000;
			PORTG &= 0b00000000;
		}
		if (bToggle)			//This is set to true only in the interrupt service routine at the bottom of the code
		{
			switch (dbState){
				case DBOOT:
					DbBOOThandler();
					break;
				case DSPEED:
					DbSPEEDhandler();
					break;
				case DTEMP:
					DbTEMPhandler();
<<<<<<< HEAD
					break;
				case DACCEL:
					DbACCELhandler();
					break;
=======
					break;
				case DACCEL:
					DbACCELhandler();
					break;
>>>>>>> 225cecc5650445554b23961d337505651e3188fd
				case DSTEER:
					DbSTEERhandler();
					break;
				default:
					break;
			}
			bToggle = 0;			// clear the flag.
		}

		switch (dbState){
			case DBOOT:
				break;
			case DSPEED:
				itoa(adcBuffer, text, 10);	//Convert the unsigned integer to an ascii string; look at 3.6 "The C programming language"
<<<<<<< HEAD
=======
				lcdGotoXY(5, 1);     //Position the cursor on
				lcdPrintData("      ", 6); //Clear the lower part of the LCD
			    lcdGotoXY(5, 1);     //Position the cursor on
				lcdPrintData(text, strlen(text)); //Display the text on the LCD
				break;
			case DTEMP:
				itoa(adcBuffer, text, 9);	//Convert the unsigned integer to an ascii string; look at 3.6 "The C programming language"
				lcdGotoXY(5, 1);     //Position the cursor on
				lcdPrintData("      ", 6); //Clear the lower part of the LCD
				lcdGotoXY(5, 1);     //Position the cursor on
				lcdPrintData(text, strlen(text)); //Display the text on the LCD
				break;
			case DACCEL:
				itoa(adcBuffer, text, 9);	//Convert the unsigned integer to an ascii string; look at 3.6 "The C programming language"
>>>>>>> 225cecc5650445554b23961d337505651e3188fd
				lcdGotoXY(5, 1);     //Position the cursor on
				lcdPrintData("      ", 6); //Clear the lower part of the LCD
			    lcdGotoXY(5, 1);     //Position the cursor on
				lcdPrintData(text, strlen(text)); //Display the text on the LCD
				break;
			case DTEMP:
				//Master receive mode, follow instruction on page 222 of the AT90CAN128 Data sheet
      			//Send start condition
				TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
				//Wait for TWINT flag to be set (Start has been transmitted)
				while ( !(TWCR & (1<<TWINT)));
				//Check the value of the TWI status register, making the pre-scaler
				debug = TWSR;
				if ((debug & 0xF8) != 0x08)	//We are master of the bus
				{
					error(debug & 0xF8);
				}
				//Load LM77 address to TWI data register
				TWDR = ((LM77_ADDR << 1)| 0x01 ); //Shift the 7 bit address while or-ing it with the read bit
				TWCR = (1 << TWINT) | (1 << TWEN); //Clear TWINT to start the transmission
				 while (!(TWCR & (1<<TWINT)));  //Wait for TWINT flag to be set which indicates that the address was sent and acknowledged
				debug = TWSR;
				if ((debug & 0xF8) != 0x40)  //SLA+R has been sent and acknowledge has been received
				{
					error(debug & 0xF8);
				}
				TWCR = (1 << TWINT) | (1 << TWEN) | (1<<TWEA); //Clear TWINT to start the reception of the first byte
				//enable acknowledge for the first byte.
				while (!(TWCR & (1<<TWINT)));  //Wait for TWINT flag to be set which indicates that the address was sent and acknowledged
				debug = TWSR;
				if ((debug & 0xF8) != 0x50)  //Data byte has been received and ACK has been sent
				{
					error(debug & 0xF8);
				}

				temp = TWDR; //High byte (D15-D8)

				TWCR = (1 << TWINT) | (1 << TWEN) ; //Clear TWINT to start the reception of the second byte
				while (!(TWCR & (1<<TWINT)));  //Wait for TWINT flag to be set which indicates that the address was sent and acknowledged
				debug = TWSR;
				if ((debug & 0xF8) != 0x58)  //Look for an acknowledgment from the LM77
				{
					error(debug & 0xF8);
				}
				tempLow = TWDR;		// Low byte (D7-D0)

				//Transmit STOP condition
				TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);

				tempLow = (tempLow>>3) | ((temp & 0x07)<<5);	// The lower 3 bits of the least significant byte do not refer to the temperature. We shift them out and need to bring in bits from the MSB
				temp = (temp >> 3);
				temperature = (temp << 8) | tempLow;


				if (temperature & 0x200)
				{
					temperature = !temperature + 1;	// two's complement
					curr_temp = temperature * (-0.5f);
				}
				else
					curr_temp = temperature * (0.5f);

				//Copy the temperature into the display[0] 16 byte character buffer
				sprintf(text,"Temp: %.1fC", curr_temp);
				lcdGotoXY(0,1);
				lcdPrintData(text, strlen(text)); //Display the text on the LCD
      break;
			case DACCEL:
				break;
			case DSTEER:
				break;
			case DSTEER:
				itoa(adcBuffer, text, 9);	//Convert the unsigned integer to an ascii string; look at 3.6 "The C programming language"
				lcdGotoXY(5, 1);     //Position the cursor on
				lcdPrintData("      ", 6); //Clear the lower part of the LCD
				lcdGotoXY(5, 1);     //Position the cursor on
				lcdPrintData(text, strlen(text)); //Display the text on the LCD
				break;

			default:
				lcdGotoXY(0, 1);     //Position the cursor on the first character of the first line
				lcdPrintData("You have a bug!", 15); //Inform of the problem
				break;
		}
	}
  return 0;
}

/* the functions below are Interrupt Service Routines, they are never called by software */

/** This function is executed every time Interrupt 6 is triggered. */
SIGNAL(SIG_INTERRUPT6)  //Execute the following code if an INT6 interrupt has been generated. It is kept short.
{
    bToggle = 1;    //Some push button has been pushed or released. Action needs to be taken.
    buttons = PINC; //Take a snapshot of the input register of Port C where the push buttons are connected.
}

ISR(ADC_vect){
    adc_value = ADCL;        //Load the low byte of the ADC result
    adc_value += (ADCH<<8); //shift the high byte by 8bits to put the high byte in the variable
}

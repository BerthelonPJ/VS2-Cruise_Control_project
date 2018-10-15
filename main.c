//
//  main.c
//  CoffeeMaker
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
#define DTOP DACCEL

enum dStates {DBOOT, DSPEED, DSTEER, DTEMP, DACCEL};    /* enumeration of states (C programming, p) */
char *dbStateName[] = {"Speed", "Steering Angle", "Temp.", "Accel."}; /* initialization of Pointer Array*/
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

/** This function initializes the TimerCounter 0 to provide a PWM to dim the display (backlight) */
void TimerCounter0setup(int start)
{
    //Setup mode on Timer counter 0 to PWM phase correct
    TCCR0A = (0<<WGM01) | (1<<WGM00);
    //Set OC0A on compare match when counting up and clear when counting down
    TCCR0A |= (1<<COM0A1) | (1<<COM0A0);
    //Setup pre-scaller for timer counter 0
    TCCR0A |= (0<<CS02) | (0<<CS01) | (1<<CS00);  //Source clock IO, no pre-scaling

    //Setup output compare register A
    OCR0A = start;
}

void TimerCounter3setup(void)
{
    // The timing of the motor to be centered is 20ms periodicity and 1.5 ms high.
    // at 16MHz, this is over 800 million count and we have on 65535 in a 16 bit counter.
    // a pre-scaler is needed. With a f/64, we get 250 kHz or 262 ms per max overflow.

    //Setup pre-scaller for timer counter 3.
    TCCR3B |= (0<<CS32) | (1<<CS31) | (1<<CS30);  //Source clock IO, /64. (Note ORing with 0 does not clear the bit)

    //Setup mode on Timer counter 3 to CTC
    TCCR3B |= (1<<WGM33) | (0<<WGM32);
    TCCR3A |= (0<<WGM31) | (0<<WGM30); // (Note ORing with 0 does not clear the bit)

    //Set OC3B on compare match when counting up and clear on Top
    TCCR3A |= (1<<COM3B1) | (0<<COM3B0);

    //Setup output compare register A
    pWidth = 188; // 1.5 ms
    OCR3B = pWidth; //
    ICR3 = 2500; // 20ms * 250 kH is the number of clock cycles/prescaler /2 for phase correct
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
// The purpose of this function is to get the current_speed of the car.
uint16_t getSpeed(void)
{
  return adc_value;
}
int cruiseControlManager(uint16_t set_speed)
{
  if (bToggle)
  {
    if ((buttons & 0b10000000) || (buttons & 0b00001000)) // If brake button or on/off button is pushed
    {
      switchCruise(1);
    }
    else // e.g if we want the cruise control manager to do its job
    {

    }
  }
}

// The purpose of this function is to enable or disable the cruise control of the car.
int switchCruise(int mode)
{
  switch (mode)
  {
    case 1: //In that case we have to disable cruise control.
      break;
    case 0: // In that case we enable againt the cruise control.
     //int current_speed=0;
	  //current_speed = getSpeed();
	  printf ("coucou");
      printf ("%d", getSpeed());
	  PINC &= 0b00000100;
      //cruiseControlManager(current_speed); // Call of the function which is in charge of the cruise control.
      break;
    default:
      break;
  }
  return 0;
}
/* This function describes how the system should react to a demand of coffee made by the user. */
int brake(void)
{
  PORTC&=0b00000010;
  switchCruise(1);
  return(0);
}

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
            brake();
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
            brake();
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
            brake();
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
            brake();
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


int main(void) {
    unsigned char temp ;        //Allocate memory for  temp
    char cursor = 0;                /* allocate a variable to keep track of the cursor position and initialize it to 0 */
    char textLine[DISPLAYLENGTH + 1];    /* allocate a consecutive array of 16 characters where your text will be stored with an end of string */
    char text[10];                //Allocate an array of 10 bytes to store text
    int adcBuffer;            // Allocate the memory to hold ADC results that is not disturbed by interrupts

    textLine[0] = 'A';                /* initialize the first ASCII character to A or 0x41 */
    textLine[1] = '\0';                /* initialize the second character to be an end of text string */

    initGPIO();        //Set up the data direction register for both ports B, C and G
    initDisplay();    //Set up the display
    initExtInt();    //Set up the external interrupt for the push buttons
    initADC();        // Setup the Analog to Digital Converter
    TimerCounter0setup(128);// enable the dimming of the display backlight with PWM using TimerCounter 0 and pin OCR0
    TimerCounter3setup();    // enable the position control of the Parallax servo motor with PWM using TimerCounter 3 and pin OCR3B JP11.INT4  (OC3B) to white wire (IO) on the motor

    ADCSRA |= (1<<ADSC);    //Start ADC
    sei();                    // Set Global Interrupts

    while(1)
    {
        if (bToggle)            //This is set to true only in the interrupt service routine at the bottom of the code
        {
            switch (dbState){
                case DBOOT:
                    DbBOOThandler();
                    break;
                case DSPEED:
                    DbSPEEDhandler();
                    break;
                case DSTEER:
                    DbSTEERhandler();
                    break;
                case DTEMP:
                    DbTEMPhandler();
                    break;
                case DACCEL:
                    DbACCELhandler();
                    break;
                default:
                    break;
            }
            bToggle = 0;            // clear the flag.
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

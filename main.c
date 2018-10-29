
//  main.c
//  Cruise Control
//
//  Created by Pierre-Jean Berthelon on 04/10/2018.
//  Copyright © 2018 Pierre-Jean Berthelon. All rights reserved.
//
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h> // input output header file for this AVR chip.
#include <avr/pgmspace.h>    // Contains some type definitions and functions.
#include <avr/interrupt.h>    // Contains ISR (Interrupt Service Routines) or interrupt handler details
#include "avrlibdefs.h"
#include "avrlibtypes.h"
#include "global.h"
#include "delay.h"
#include "lcd.h"
#include "gpio.h"
#include "usart.h"
#include "spi.h"
#include "lm77.h"
#include "i2c.h"
#include "i2cconf.h"
#include "realtime_clock.h"


#define DISPLAYLENGTH 16
#define DTOP DSTEER

//Poll the temperature each 500ms
#define COUNTER_POLL_TEMP_INTERVAL	500
//Poll the timz each 100ms
#define COUNTER_POLL_TIME_INTERVAL	100

// I2C address of the LM77 temperature sensor
#define LM77_ADDR	0x48

//Counter which keeps track of how many ms has passed since the last temperature polling
volatile unsigned int counter_poll_temperature = COUNTER_POLL_TEMP_INTERVAL;
//Counter which keeps track of how many ms has passed since the last time polling
volatile unsigned int counter_poll_time = COUNTER_POLL_TIME_INTERVAL;

//initialize cruise control state to 0, e.g is disabled, will be 1 when enable.
volatile int controlState = 0;

// enumeration of states that will be displayed on the second line of the screen
enum dStates {DBOOT,DSPEED,DTEMP,DACCEL,DICP,DSTEER};
// Strings that will be displayed on the second line of the screen
char *dbStateName[] = {"Cruise Control","Speed", "Temp.", "Accel.","ICP","Steer"};
// Int used to go through the array of states
volatile unsigned int dbState;

// This registers holds a copy of PINC when an external interrupt 6 has occurred.
volatile unsigned char buttons;
// This registers is a boolean that is set when an interrupt 6 occurs and cleared when serviced in the code.
volatile unsigned char bToggle = 0;

//Allocate the double byte memory space into which the result of the 10 bits Analog to Digital Converter (ADC) is stored.
volatile uint16_t adc_value;
//Allocate the double byte memory space into which the result of the 10 bits Analog to Digital Converter (ADC) is stored.
volatile uint16_t adc_buffer;
// This variable is used to know wether the cruise controller has been initialised or not
volatile int initControl = 0;

// Floating type memory allocation for the temperature
volatile float curr_temp = 0;

// Pulse width for the servo motor on Timer Counter 3
volatile int pWidth;

// Var used to save the value given to the cruise controller.
volatile int set_speed;

volatile int init_speed = 0;

// SPI new byte
volatile char spiByte;

//Input capture register
volatile uint16_t curr_icp, last_icp;
// Boolean to keep track if Timer/Counter has overflown
volatile unsigned int overflow;



/** Function used to initialize the ADC on the board. */
int initADC()
{
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
    // Set the direction of the IO pins on Port C to output on the 3 least significant bits and input on the 5 higher ones. 5 buttons and 3 LEDs.
    DDRC = 0b00000111;
    DDRG |= 0b00000011;

    //Any logical change to INT6 generates an interrupt
    // This can be changed, see table 10-1 p.94
    EICRB |= (0<<ISC61) | (1<<ISC60);
    //Enable interrupt on INT6
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
     //initialize the LCD
    lcdInit();
    //clear the LCD
    lcdClear();
     //go to the home of the LCD
    lcdHome();
    //Display the text on the LCD
    lcdPrintData(dbStateName[dbState], strlen(dbStateName[dbState]));
    // Turn on the display's back light.
    PORTB |= 1 << DISPLAY_LED;
    return(0);
}

/** This function initializes the TimerCounter 0 to provide a PWM to dim the display (backlight) */
void TimerCounter0setup(int start)
{
	//Setup mode on Timer counter 0 to PWM phase correct, see table 12-1 p.110
	TCCR0A = (0<<WGM01) | (1<<WGM00);
	//Set OC0A on compare match when counting up and clear when counting down, table 12-2 p.110
	TCCR0A |= (1<<COM0A1) | (1<<COM0A0);
	//Setup pre-scaller for timer counter 0, table 12-5 p.111
	TCCR0A |= (0<<CS02) | (0<<CS01) | (1<<CS00);  //Source clock IO, no pre-scaling

	//Setup output compare register A e.g velue max that counter will go.
	OCR0A = start;
}

/** Initialize the timerCounter3, which is used to control the servo motor */
void TimerCounter3setup(void)
{
	// The timing of the motor to be centered is 20ms periodicity and 1.5 ms high.
	// at 16MHz, this is over 800 million count and we have on 65535 in a 16 bit counter.
	// a pre-scaler is needed. With a f/64, we get 250 kHz or 262 ms per max overflow.

	//Setup pre-scaller for timer counter 3, table 13-5 p.139
	TCCR3B |= (0<<CS32) | (1<<CS31) | (1<<CS30);  //Source clock IO, /64. (Note ORing with 0 does not clear the bit)

	//Setup mode on Timer counter 3 to PWM, phase and frequency correct. table 13-4 p.138.
	TCCR3B |= (1<<WGM33) | (0<<WGM32);
	TCCR3A |= (0<<WGM31) | (0<<WGM30); // (Note ORing with 0 does not clear the bit)

	//Set OC3B on compare match when counting up and clear on Top, table 13-3 p. 137
	TCCR3A |= (1<<COM3B1) | (0<<COM3B0);

	//Setup output compare register A
  // 1.5 ms
  pWidth = 188;
  //value of the comparing reference set to pWidth, bottom of page 141
	OCR3B = pWidth;
  // 20ms * 250 kH is the number of clock cycles/prescaler /2 for phase correct
  //Used for input capture ??
  ICR3 = 2500;
}

/** This function initializes the TimerCouter1 for Input Capture of a cyclic wave.*/
void initializeTimerCounter1(void)
{
	// The input capture pin is on JP14 near the relay on the development board.
	//Page 137 or 138
	TCCR1A = 0;				//Timer/Counter1 Control Register A is not really used
	TCCR1B = (1<<ICNC1);	//noise canceler
	TCCR1B |= (1<<ICES1);	//rising edge on ICP1 cause interrupt
	TCCR1B |= (0<<CS12)| (0<<CS11)| (1<<CS10); //NO pre-scaler,
	TIFR1 |= (1<<ICF1);		//Set input capture flag
	TCNT1 = 0;				//Timer Counter 1
	TIMSK1 |= (1<<ICIE1); //|(1<<TOIE1); //Timer Interrupt Mask Register,Input Capture Interrupt Enable, T/C overflow Interrupt commented out so overflow counter is not incremented
}

/** This function is called when cycling through the states.*/
int dbStateUp(void)
{
    // Used to go back to the begining of the array if at the end
    if (++dbState > DTOP)
        dbState = DSPEED;

    // Just displaying the new current state on the LCD display
    lcdClear();
    lcdGotoXY(0,1);
    lcdPrintData(dbStateName[dbState], strlen(dbStateName[dbState])); //Display the text on the LCD
    return 0;
}

/** This function is called only when it's the first time we set the cruise control to init the set speed etc... */
void initCruise(uint16_t select_speed)
{
  //Just puts the servo motor to the middle of its capacity.
  char sreg;
  pWidth = 188;			// pulse width 188 is 21.5ms per cycle; return to center
  cli();					//disable the interrupts and save the status register when operating on 16 variables.
  //saving value of SREG in a variable
  sreg = SREG;
  // Set the angle on the servo
  OCR3B = pWidth;		// 16 bits operation
  //restore the status register and re-enable interrupts
  SREG  = sreg;
  sei();
  //Basic operation to set the speed that cruise control should follow.
  // This value should be between 65 and 305 regarding to the values we can get from the ADC ( potentiometer)
  set_speed = (select_speed + 277)/4.26;
}

/** This function is called when the cruise controller is on. it controls the cruise manager,
The purpose is that we retrieve the current speed from the potentiometer, then we transform it as in the previous function,
and finally we compare it to the value we set earlier.
If the current speed is lower than expected, then we open the throttle valve. To the oposite we may close the valve to reduce speed.
And if value is as expected, then we shall not move the valve. */
void cruiseController(void)
{
  //retrieve the current speed from the potentiometer
  //disable ADC interrupt to prevent value update during the conversion
  ADCSRA &= ~(1<<ADIE);
  //get the ADC value and store it in instant_speed
  uint16_t instant_speed = adc_value;
  //re-enable ADC interrupt
  ADCSRA |= (1<<ADIE);
  // using the top 8 bits of the ADC, load OCR0A to compare to the timer Counter 0 to generate aPWM for the display back light
  OCR0A = instant_speed >> 2;

  char sreg;
  //Converting the current speed value to be comparable to the set speed
  instant_speed=(instant_speed + 277)/4.26;
  //If the current speed equals the set speed, nothing to do but just turning led5 on.
  if (set_speed == instant_speed)
  {
    PORTC |= 0b00000001;
  }
  //If the current speed is superior to the set speed then we close the throttle valve.
  else if (set_speed < instant_speed)
  {
    //Switch led 5 off
    PORTC &= 0b11111110;

    //If we haven't reached the lower border yet, then we can close the valve
    if (pWidth > 80)
    {
      //Decrease the angle of the servo motor by the value of -1/2500
      pWidth -= 2;
  	  cli();
    	sreg = SREG;
    	OCR3B = pWidth;
    	SREG  = sreg;
    	sei();
    }
  }
  //If the current speed is inferior to the set speed then we open the throttle valve.
  else if (set_speed > instant_speed)
  {
    //Switch led 5 off
    PORTC &= 0b11111110;

    //If we haven't reached the higher border yet, then we can open valve
    if (pWidth < 260)
    {
      //Increase the angle of the servo motor by 2/2500 ms
      pWidth += 2;
    	cli();
    	sreg = SREG;
    	OCR3B = pWidth;
    	SREG  = sreg;
    	sei();
    }
  }
}
/** The purpose of this function is to handle the switch of modes ( enable or disable cruise controller) */
void switchControl(int control)
{
  //If we want to turn off the controller
  if (control == 0)
  {
    controlState = 0;
    //we turn off the led 1
    PORTG &= 0b11111110;
    PORTC &= 0b11111110;
  }
  //Or if we want to turn on the controller
  else if (control == 1)
  {
    //Light up led 1
    PORTG |= 0b00000001;
    controlState = 1;
  }
}
/** This function will be called when the brake button is pushed ( central button )
Its purpose is to stop instantly the cruise controller, in case of emergency braking for example. */
void brake(void)
{
  char sreg;
  //We just close completly the throttle valve.
  pWidth = 60;
  cli();
  sreg = SREG;
  OCR3B = pWidth;
  SREG  = sreg;
  sei();

  //Supposed to turn on led 2 as long as we push on the button,
  // Need to be fixed,
  // Should also turn off the cruise controller.
  PORTG |= 0b00000010;
  delay_ms(250);
  PORTG &= 0b11111101;
  PORTC &= 0b11111110;
  switchControl(0);

}
/** This function uses the push buttons to let the user to change states upon boot up. It is also used to enter in Coffee maker mode*/
int DbBOOThandler(void)
{
  switch(buttons & 0b11111000)
  {
      //S5 center button
      case 0b10000000:
          brake();
          break;
      //S4  upper button
      case 0b01000000:
          //Increase set speed of the cruise controller.
          set_speed++;
          break;
      //S3 left button
      case 0b00100000:
          //cycle through the menu
          dbStateUp();
          break;
      //S2 lower button
      case 0b00010000:
          //Decrease set speed of the cruise controller.
          set_speed--;
          break;
      //S1 right button
      case 0b00001000:
      //This button enables or disables the cruise controller, wether it is enabled or disabled.
          if (controlState == 1 )
          {
            switchControl(0); }
          else if (controlState == 0)
          {
            switchControl(1);
          }
          break;
      default:
          /* do nothing */
          break;
  }
  return 0;
}
/** This function uses the push buttons to let the user to change states upon boot up */
int DbSPEEDhandler(void)
{
  switch(buttons & 0b11111000)
  {
      //S5 center button
      case 0b10000000:
          brake();
          break;
      //S4  upper button
      case 0b01000000:
          //Increase set speed of the cruise controller.
          set_speed++;
          break;
      //S3 left button
      case 0b00100000:
          //cycle through the menu
          dbStateUp();
          break;
      //S2 lower button
      case 0b00010000:
          //Decrease set speed of the cruise controller.
          set_speed--;
          break;
      //S1 right button
      case 0b00001000:
      //This button enables or disables the cruise controller, wether it is enabled or disabled.
          if (controlState == 1 )
          {
            switchControl(0); }
          else if (controlState == 0)
          {
            switchControl(1);
          }
          break;
      default:
          /* do nothing */
          break;
  }
  return 0;
}
/** This function uses the push buttons to let the user to change states upon boot up */
int DbSTEERhandler(void)
{
  switch(buttons & 0b11111000)
  {
      //S5 center button
      case 0b10000000:
          brake();
          break;
      //S4  upper button
      case 0b01000000:
          //Increase set speed of the cruise controller.
          set_speed++;
          break;
      //S3 left button
      case 0b00100000:
          //cycle through the menu
          dbStateUp();
          break;
      //S2 lower button
      case 0b00010000:
          //Decrease set speed of the cruise controller.
          set_speed--;
          break;
      //S1 right button
      case 0b00001000:
      //This button enables or disables the cruise controller, wether it is enabled or disabled.
          if (controlState == 1 )
          {
            switchControl(0); }
          else if (controlState == 0)
          {
            switchControl(1);
          }
          break;
      default:
          /* do nothing */
          break;
  }
  return 0;
}
/** This function uses the push buttons to let the user to change states upon boot up */
int DbTEMPhandler(void)
{
  switch(buttons & 0b11111000)
  {
      //S5 center button
      case 0b10000000:
          brake();
          break;
      //S4  upper button
      case 0b01000000:
          //Increase set speed of the cruise controller.
          set_speed++;
          break;
      //S3 left button
      case 0b00100000:
          //cycle through the menu
          dbStateUp();
          break;
      //S2 lower button
      case 0b00010000:
          //Decrease set speed of the cruise controller.
          set_speed--;
          break;
      //S1 right button
      case 0b00001000:
      //This button enables or disables the cruise controller, wether it is enabled or disabled.
          if (controlState == 1 )
          {
            switchControl(0); }
          else if (controlState == 0)
          {
            switchControl(1);
          }
          break;
      default:
          /* do nothing */
          break;
  }
  return 0;
}
/** This function uses the push buttons to let the user to change states upon boot up */
int DbACCELhandler(void)
{
  switch(buttons & 0b11111000)
  {
      //S5 center button
      case 0b10000000:
          brake();
          break;
      //S4  upper button
      case 0b01000000:
          //Increase set speed of the cruise controller.
          set_speed++;
          break;
      //S3 left button
      case 0b00100000:
          //cycle through the menu
          dbStateUp();
          break;
      //S2 lower button
      case 0b00010000:
          //Decrease set speed of the cruise controller.
          set_speed--;
          break;
      //S1 right button
      case 0b00001000:
      //This button enables or disables the cruise controller, wether it is enabled or disabled.
          if (controlState == 1 )
          {
            switchControl(0); }
          else if (controlState == 0)
          {
            switchControl(1);
          }
          break;
      default:
          /* do nothing */
          break;
  }
  return 0;
}
/** This function uses the push buttons to let the user to change states upon boot up */
int DbTIMEhandler(void)
{
  switch(buttons & 0b11111000)
  {
      //S5 center button
      case 0b10000000:
          brake();
          break;
      //S4  upper button
      case 0b01000000:
          //Increase set speed of the cruise controller.
          set_speed++;
          break;
      //S3 left button
      case 0b00100000:
          //cycle through the menu
          dbStateUp();
          break;
      //S2 lower button
      case 0b00010000:
          //Decrease set speed of the cruise controller.
          set_speed--;
          break;
      //S1 right button
      case 0b00001000:
      //This button enables or disables the cruise controller, wether it is enabled or disabled.
          if (controlState == 1 )
          {
            switchControl(0); }
          else if (controlState == 0)
          {
            switchControl(1);
          }
          break;
      default:
          /* do nothing */
          break;
  }
  return 0;
}

int main(void) {

  char temp, tempLow ;		//Allocate memory for  temp and temp2
  //allocate a variable to keep track of the cursor position and initialize it to 0
  char cursor = 0;
  //Allocate an array of 10 bytes to store text
  char text[10];
  //Allocate an array of 10 bytes to store text_speed
  char text_speed[10];
  //Two dimensionnal array representing the LCD display
  char display[2][16];

  //debug byte to look at the TWI status register code
  char debug;

  // allocate memory for elapse time input capture events
  int etime;


  // allocate the memory for the 3D accelerometer structure
  accelSPI axel3D;

  //Set up the data direction register for both ports B, C and G
  initGPIO();
  //Set up the display
  initDisplay();
  //Set up the external interrupt for the push buttons
  initExtInt();
  // Setup the Analog to Digital Converter
  initADC();
  //Initialize USART 1 with BAUDRATE_19200 (look at pages 183 and 203)
  usart1_init(51);
  //Initialize TimerCounter0 with a compare value of 128
  // enable the dimming of the display backlight with PWM using TimerCounter 0 and pin OCR0
  TimerCounter0setup(128);
  //Initialise TimerCounter1 to use PWM for inpiut capture.
  initializeTimerCounter1();
  //Initialize TimerCounter3 to use PWM on the servo motor.
  TimerCounter3setup();
  // Set up SPI and accelerometer
  SPI_MasterInit();
  //accelerometer initialization
  // Write at address, 0x16 = 0b00010110 shifted one to the left and the MSB is 1 to write
  // --,DRPD,SPI3W,STON,GLVL[1],GLVL[0],MODE[1],MODE[0]
  // GLVL [1:0] --> 0 1 --> 2g range, 64 LSB/g
  // MODE [1:0] --> 0 1 --> measurement mode
  SPI_MasterTransmit(0b10101100,0b00000101);
  //Disable global interrupts
  cli();
  //Initialize the I2C communication bus
  i2cInit();
  //Start ADC
  ADCSRA |= (1<<ADSC);
  // Set Global Interrupts
  sei();
  //Clear the display array
	for (int y=0;y<2;y++)
		for (int x=0;x<16;x++)
			display[y][x] = 0;

  /* Initialize the RTC, arguments are:
  Seconds, minutes, hours, day, month, year */
  //rtc_init(10,24,10,3,6,16);
  while(1)
	{
    // Just retrieve the ADC value and store it into the curr_speed variable
    //disable ADC interrupt to prevent value update during the conversion
    ADCSRA &= ~(1<<ADIE);
    uint16_t curr_speed = adc_value;
    //re-enable ADC interrupt
    ADCSRA |= (1<<ADIE);
    // using the top 8 bits of the ADC, load OCR0A to compare to the timer Counter 0 to generate a PWM for the display back light
    OCR0A = curr_speed >> 2;

    //This is set to true only in the interrupt service routine at the bottom of the code
		if (bToggle)
		{
      // A essayer comme ça, pas sûr que ça marche mais on sait jamais.
      DbBOOThandler();
			bToggle = 0;			// clear the flag.
		}
    //Goes in if the cruise control is activated
    if (controlState == 1)
    {
      //If this is the first time we launch the cruise control
      if (initControl == 0)
      {
        //Initialize the set speed in this function
        initCruise(curr_speed);
        // This is to avoid looping again and again in this initialisation process
        initControl=1;
      }
      //Call this function that describe the cruise controller reaction.
      cruiseController();
    }
    //Now depending on the mode we are in, the system will not display the same thing on the display
		switch (dbState)
    {
			case DBOOT:
        if ( controlState == 1 )
        {
          //Convert set_speed to a decimal value that we can display on the display
          itoa(set_speed,text_speed, 10);
          // We print the result in array "display"
          sprintf(display[0],"set speed: %s",text_speed);
          //Pointer of the screen goes back to the upper left corner
          lcdHome();
          //Clear the first line
          lcdPrintData("                ",16);
          //Pointer of the screen goes back to the upper left corner
          lcdHome();
          //We print the content of the first line of the array "display" on the LCD display
          lcdPrintData(display[0], strlen(display[0]));
          //We send the same array through the USART1, which displays it on a laptop temrinal
          usart1_sendstring(display[0], strlen(display[0]));
          // New line on the USART
          usart1_transmit('\n');
          // transmit a carriage return
          usart1_transmit('\r');
        }
      	break;
			case DSPEED:
         //Need to init the ADC each time, because when I use steer (second ADC channel) changes parameters
		     initADC();
         // A essayer sans
         ADCSRA &= ~(1<<ADIE);		//disable ADC interrupt to prevent value update during the conversion
         uint16_t curr_speed = adc_value;
         ADCSRA |= (1<<ADIE);		//re-enable ADC interrupt

        //Only execute this if we are using the cruise controller
        if ( controlState == 1 )
        {
          itoa(set_speed,text_speed, 10);
          sprintf(display[0],"set speed: %s",text_speed);
          lcdHome();
          lcdPrintData("                ",16);
          lcdHome();
          lcdPrintData(display[0], strlen(display[0]));
          usart1_sendstring(display[0], strlen(display[0]));	/* transmit over USART 1 the value of the ADC */
          usart1_transmit('\t');	// transmit a new line
        }

        curr_speed = (curr_speed + 277)/4.26;
				itoa(curr_speed, text, 10);	//Convert the unsigned integer to an ascii string; look at 3.6 "The C programming language"
        sprintf(display[1],"Curr. speed: %s",text);
        lcdGotoXY(0, 1);     //Position the cursor on
				lcdPrintData("                ", 16); //Clear the lower part of the LCD
			  lcdGotoXY(0, 1);     //Position the cursor on
				lcdPrintData(display[1], strlen(display[1])); //Display the text on the LCD

        usart1_sendstring(display[1], strlen(display[1]));	/* transmit over USART 1 the value of the ADC */
        usart1_transmit('\n');
        usart1_transmit('\r');	// transmit a carriage return
        break;
			case DTEMP:

         if (counter_poll_temperature >= COUNTER_POLL_TEMP_INTERVAL)
         {
            //Retrieve the temperature from the lm77 temperature sensor
            curr_temp = lm77_read_temp();


            //Copy the temperature into the display[0] 16 byte character buffer
            sprintf(display[1],"Temp: %.1fC",curr_temp);
            // usart1_sendstring(display[1], strlen(display[1]));

            usart1_sendstring(display[1], strlen(display[1]));	/* transmit over USART 1 the value of the ADC */
            usart1_transmit('\t');	// transmit a new line
            usart1_sendstring(display[0], strlen(display[0]));	/* transmit over USART 1 the value of the ADC */
            usart1_transmit('\n');
            usart1_transmit('\r');	// transmit a carriage return

            //If the temperature goes over 28 degrees the relay will get activated. When it
            //goes under 28 degrees the relay will be deactivated

            lcdGotoXY(0,1);
            lcdPrintData("                ",16);
            lcdGotoXY(0,1);
            lcdPrintData(display[1],strlen(display[1]));

            //Reset the counter variable
            counter_poll_temperature = 0;
          }

          break;
			case DACCEL:


        axel3D = acc(); // get the new averages of x, y and z from the accelerometer

        if (axel3D.x & 0x80)// if value is negative (can be corrected with two's complement
           (~axel3D.x)+1;// [value is x, y or z]

        utoa(axel3D.x, text, 10);	//Convert the unsigned integer to an ascii string; look at 3.6 "The C programming language"
        if ( controlState == 1 )
        {
          itoa(set_speed,text_speed, 10);
          sprintf(display[0],"set speed: %s",text_speed);
          lcdHome();
          lcdPrintData("                ",16);
          lcdHome();
          lcdPrintData(display[0], strlen(display[0]));
        }

        sprintf(display[1], "Curr. Accel :%s", text);

        lcdGotoXY(0, 1);     //Position the cursor on
        lcdPrintData("                ", 16); //Clear part of the lower part of the LCD
        lcdGotoXY(0, 1);     //Position the cursor
        lcdPrintData(display[1], strlen(display[1])); //Display the text on the LCD

        usart1_sendstring(display[1], strlen(display[1]));	/* transmit over USART 1 the value of the ADC */
        usart1_transmit('\t');	// transmit a new line
        usart1_sendstring(display[0], strlen(display[0]));	/* transmit over USART 1 the value of the ADC */
        usart1_transmit('\n');
        usart1_transmit('\r');	// transmit a carriage return

				break;
      //Not Working, but don't really know why...
      // case DTIME:
      //   //Function will update the time at COUNTER_POLL_TIME_INTERVAL intervals (defined at the top of this file)
      //   if (counter_poll_time >= COUNTER_POLL_TIME_INTERVAL) {
      //     char temp_time[8];
      //     rtc_get_time((char *)temp_time);
      //
      //     sprintf(display[1],"Time: %s",temp_time);
      //     lcdGotoXY(0,1);
      //     lcdPrintData(display[1],strlen(display[1]));
      //     //Reset the counter variable
      //     counter_poll_time = 0;
      //   }
      //   break;
      case DICP:
				cli();
				if (curr_icp > last_icp)
				{
					etime = curr_icp - last_icp;
				}
				else
				{
					etime = curr_icp + (65365 - last_icp);
				}
				sei();
        if ( controlState == 1 )
        {
          itoa(set_speed,text_speed, 10);
          sprintf(display[0],"set speed: %s",text_speed);
          lcdHome();
          lcdPrintData("                ",16);
          lcdHome();
          lcdPrintData(display[0], strlen(display[0]));
        }
				// If you used the overflow counter, you would add them here and need to reset the overflow counter.

				utoa(etime, &text[0], 10);	//Convert the unsigned integer to an ascii string
        sprintf(display[1], "Curr. dist :%s", text);
        lcdGotoXY(0, 1);     //Position the cursor on
        lcdPrintData("                ", 16); //Clear part of the lower part of the LCD
        lcdGotoXY(0, 1);     //Position the cursor
        lcdPrintData(display[1], strlen(display[1])); //Display the text on the LCD

        usart1_sendstring(display[1], strlen(display[1]));	/* transmit over USART 1 the value of the ADC */
        usart1_transmit('\t');	// transmit a new line
        usart1_sendstring(display[0], strlen(display[0]));	/* transmit over USART 1 the value of the ADC */
        usart1_transmit('\n');
        usart1_transmit('\r');	// transmit a carriage return
				break;

			case DSTEER:
        //Basically the same that what is in initADC'), but we set the ADC channel to channel 1 instead of channel 0
        ADMUX &= ~(1<<REFS1);  //Clear REFS1 (although it should be 0 at reset)
        ADMUX |= (1<<REFS0);   //Set REFS0

        ADMUX |= (0b11100001);

        ADMUX &= ~(1<<ADLAR);  //Making sure ADLAR is zero (somehow it was set to 1)
        //The ACDC control and status register B ADCSRB
        ADCSRB &= ~(1<<ADTS2) & ~(1<<ADTS1) & ~(1<<ADTS0);  //Free running mode
        //The ADC control and status register A ADCSRA
        ADCSRA |= (1<<ADPS2) | (1<<ADPS1) |(1<<ADPS0);//set sampling frequency pre-scaler to a division by 128
        ADCSRA |= (1<<ADEN)  | (1<<ADATE) | (1<<ADIE);//enable ADC, able ADC auto trigger, enable ADC interrupt
        ADCSRA &= ~(1<<ADIE);		//disable ADC interrupt to prevent value update during the conversion
        adc_buffer = adc_value;
        ADCSRA |= (1<<ADIE);		//re-enable ADC interrupt

        OCR0A = adc_buffer >> 2;		// using the top 8 bits of the ADC, load OCR0A to compare to the timer Counter 0 to generate aPWM for the display back light

        //Executed only if the cruise controller is enabled
        if ( controlState == 1 )
        {
          itoa(set_speed,text_speed, 10);
          sprintf(display[0],"set speed: %s",text_speed);
          lcdHome();
          lcdPrintData("                ",16);
          lcdHome();
          lcdPrintData(display[0], strlen(display[0]));
          usart1_sendstring(display[0], strlen(display[0]));	/* transmit over USART 1 the value of the ADC */
          usart1_transmit('\t');	// transmit a new line
        }
        adc_buffer = adc_buffer / 11.367;
				itoa(adc_buffer, text, 10);	//Convert the unsigned integer to an ascii string; look at 3.6 "The C programming language"
        sprintf(display[1],"Steer: %s",text);
        lcdGotoXY(0, 1);     //Position the cursor on
        lcdPrintData("                ", 16); //Clear the lower part of the LCD
        lcdGotoXY(0, 1);     //Position the cursor on
        lcdPrintData(display[1], strlen(display[1])); //Display the text on the LCD

        usart1_sendstring(display[1], strlen(display[1]));	/* transmit over USART 1 the value of the ADC */

        usart1_transmit('\n');
        usart1_transmit('\r');	// transmit a carriage return

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
    if (PINC & (1<<7))
    {
      brake();
    }
    bToggle = 1;    //Some push button has been pushed or released. Action needs to be taken.
    buttons = PINC; //Take a snapshot of the input register of Port C where the push buttons are connected.
}

ISR(ADC_vect){
    adc_value = ADCL;        //Load the low byte of the ADC result
    adc_value += (ADCH<<8); //shift the high byte by 8bits to put the high byte in the variable
}

/*!Output compare 0 interrupt - "called" with 1ms intervals*/
ISR(SIG_OUTPUT_COMPARE0) {
	counter_poll_temperature++;
	counter_poll_time++;
}

// ISR(USART1_RX_vect){
// 	char ReceivedByte;
// 	ReceivedByte = UDR1;
// 	UDR1 = ReceivedByte;
// 	if (uCursor < DISPLAYLENGTH){
// 		echoString[uCursor] = ReceivedByte;
// 		echoString[++uCursor] = '\0';
// 	} else {
// 		uCursor = 0;	// reset the cursor to the beginning of the line
// 		echoString[uCursor] = '\0';
// 	}
// }
SIGNAL(SPI_STC_vect) {		//interrupt from SPI
	spiByte = SPDR;	// Read - and store incoming byte
}

/** This function is executed every time an event. */
ISR(TIMER1_CAPT_vect)
{
	last_icp = curr_icp;
	curr_icp = ICR1;
}

/** This function is executed every time Interrupt 6 is triggered. */
ISR(TIMER1_OVF_vect)
{
	overflow++;
}

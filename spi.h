/*
 * spi.h
 *
 * Created: 8/24/2018 10:31:43 AM
 *  Author: Jan van Deventer
 * The purpose of these function is to introduce the on board accelerometer which we interact with via SPI.
 * Updated on August 29, 2018.
 */

#define DDR_SPI DDRB
#define DD_MISO PB3
#define DD_MOSI PB2
#define DD_SCK PB1
#define DD_SS PB0	// This pin in only connected to JP12, so one always needs a jumper
// if using more than one slave, one needs more chip select

#define link 46 // elbow to shoulder length

// This is a data structure with four attributes: the accelerations in three directions and a status register form the chip
typedef struct
{
	int x,y,z;
	char statReg;
} accelSPI;

void SPI_MasterInit(void);

void SPI_MasterTransmit(uint8_t rwAddress,uint8_t cData);

accelSPI acc(void);

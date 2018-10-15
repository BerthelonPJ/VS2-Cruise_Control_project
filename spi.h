/*
 * spi.h
 *
 * Created: 8/24/2018 10:31:43 AM
 *  Author: Jan van Deventer
 * The purpose of these function is to introduce the students to SPI.
 * In this case, we start using it in between boards rather than in between components on a board.
 */ 

#define DDR_SPI DDRB
#define DD_MISO PB3
#define DD_MOSI PB2
#define DD_SCK PB1
#define DD_SS PB0

void SPI_MasterInit(void);

void SPI_MasterTransmit(char cData);

void SPI_SlaveInit(void);

char SPI_SlaveReceive(void);

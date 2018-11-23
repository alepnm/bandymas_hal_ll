#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED

#include "defs.h"

void    SPI_Transmit8(uint8_t* txdata, uint16_t len);
void    SPI_Receive8(uint8_t* rxdata, uint16_t len);
void    SPI_TransmitReceive8(uint8_t* txdata, uint8_t* rxdata, uint16_t len);

#endif /* SPI_H_INCLUDED */

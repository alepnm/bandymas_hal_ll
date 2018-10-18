#ifndef	_EEPROM_24XX_H
#define	_EEPROM_24XX_H


#include "stepper.h"


uint8_t     EEP24XX_IsConnected( void );
uint8_t     EEP24XX_IsBusy( void );
uint8_t	    EEP24XX_Write( uint16_t addr, void* data, size_t size_of_data );
uint8_t	    EEP24XX_Read( uint16_t addr, void* data, size_t size_of_data );
uint8_t     EEP24XX_Clear( void );


#endif


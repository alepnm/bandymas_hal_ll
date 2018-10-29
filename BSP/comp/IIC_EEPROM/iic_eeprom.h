#ifndef	_EEPROM_24XX_H
#define	_EEPROM_24XX_H

#include <stdint.h>

#define     EE24LC08

#ifdef EE24LC08
/* 24LC08 EEPROM */
#define     I2C_EEP_BASE_ADDRESS    0x50
#define     I2C_MEMORY_SIZE         1024
#define     PAGE_CALC_SHIFT_VAL     4
#define     BLOCK_CALC_SHIFT_VAL    8
#define     DATA_OFFSET_MASK        0x0F
#define     PAGE_SIZE               16
#define     PAGES_IN_BLOCK          16
#endif

#ifdef EE24LC16
/* 24LC16 EEPROM */
#define     I2C_EEP_BASE_ADDRESS    0x50
#define     I2C_MEMORY_SIZE         2048
#define     PAGE_CALC_SHIFT_VAL     4
#define     BLOCK_CALC_SHIFT_VAL    8
#define     DATA_OFFSET_MASK        0x0F
#define     PAGE_SIZE               16
#define     PAGES_IN_BLOCK          16
#endif

uint8_t	    EEP24XX_Write( uint16_t addr, void* data, size_t size_of_data );
uint8_t	    EEP24XX_Read( uint16_t addr, void* data, size_t size_of_data );
uint8_t     EEP24XX_Clear( void );


#endif


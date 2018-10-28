#ifndef DEFS_H_INCLUDED
#define DEFS_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


typedef enum { RES_OK = 0, RES_ERROR, RES_BUSY, RES_TIMEOUT, RES_BAD_PARAMS } eRESULT_TypeDef;  // atitinka HAL_StatusTypeDef

/*  SYSTEM DEFAULTS */
#define     MODBUS_ENABLE

#define     MB_PORT_DEF                 ( 0u )
#define     MBADDR_DEF                  ( 10u )     //0x0A
#define     MBPARITY_DEF                MB_PAR_NONE
#define     MBBAURATE_DEF               ( 3u )      // bodreito indeksas lenteleje ( 3->19200 )
#define     MBSTOPBITS_DEF              ( 1u )
#define     MBWORDLENGHT_DEF            ( 8u )
#define     SOUND_LEVEL_DEF             SND_OFF
#define     WDT_FUNC_DEF                DISABLE

/* UART */
typedef struct {

    uint8_t         Uart;
    uint8_t         ModbusActive;

    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }MbAddr;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;      // aktyvi reiksme
    }Baudrate;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }Parity;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }StopBits;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }DataBits;
} MbPortParams_TypeDef;

extern MbPortParams_TypeDef MbPortParams;


#endif /* DEFS_H_INCLUDED */

#ifndef DEFS_H_INCLUDED
#define DEFS_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#ifndef NULL
    #define NULL ((void *)0)
#endif

#ifndef TRUE
    #define TRUE        (1u)
#endif

#ifndef FALSE
    #define FALSE       (0u)
#endif

#define DUMMY  0
#define UNUSED(x) ((void)(x))

#define LO8(x)  (uint8_t)( x & 0x00FF )
#define HI8(x)  (uint8_t)((x & 0xFF00 ) >> 8)
#define LO16(x) (uint16_t)( x & 0x0000FFFF )
#define HI16(x) (uint16_t)((x & 0xFFFF0000 ) >> 16)

typedef uint8_t byte;

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

#define     EEADR_INITBYTE              10  // 1 baitas
#define     EEADR_MBPORTPARAMS          11  // 44 baitu
#define     EEADR_RTCTIME               56  // 8 baitai
#define     EEADR_RTCDATE               64  // 4 baitai


#define     EE_INITBYTE_DEF             0xAA

/* UART */
#pragma pack(push, 1)
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
#pragma pack(pop)

extern MbPortParams_TypeDef MbPortParams;


#endif /* DEFS_H_INCLUDED */

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
#define UNUSED(x)       ((void)(x))

#define LO8(x)          (uint8_t)( x & 0x00FF )
#define HI8(x)          (uint8_t)((x & 0xFF00 ) >> 8)
#define LO16(x)         (uint16_t)( x & 0x0000FFFF )
#define HI16(x)         (uint16_t)((x & 0xFFFF0000 ) >> 16)

typedef uint8_t         byte;

typedef enum { RES_OK = 0, RES_ERROR, RES_BUSY, RES_TIMEOUT, RES_BAD_PARAMS } eRESULT_TypeDef;


/*  SYSTEM DEFAULTS */
//#define     MODBUS_ENABLE

#define     SOUND_LEVEL_DEF             SND_OFF
#define     WDT_FUNC_DEF                DISABLE


/* EEPROM bazinis adresas */
#define     EEADR_BASE                  10

/* EEPROM adresu offsetai */
#define     EEADR_INITBYTE              EEADR_BASE+10  // 1 baitas
#define     EEADR_MBPORTPARAMS          EEADR_BASE+11  // 44 baitai
#define     EEADR_RTCTIME               EEADR_BASE+56  // 8 baitai
#define     EEADR_RTCDATE               EEADR_BASE+64  // 4 baitai


#define     EE_INITBYTE_DEF             0xAA


void SystickDelay_ms(uint32_t delay);
void Delay_ms(uint32_t delay);

#endif /* DEFS_H_INCLUDED */

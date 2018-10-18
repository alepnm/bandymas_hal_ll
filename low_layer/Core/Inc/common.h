#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef NULL
    #define NULL ((void *)0)
#endif

#ifndef TRUE
    #define TRUE        (1u)
#endif

#ifndef FALSE
    #define FALSE       (0u)
#endif

#define LO16(x) (uint16_t)( x & 0x0000FFFF )
#define HI16(x) (uint16_t)((x & 0xFFFF0000 ) >> 16)

#define DUMMY  0

typedef uint8_t byte;

#endif /* COMMON_H_INCLUDED */

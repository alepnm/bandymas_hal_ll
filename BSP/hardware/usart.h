#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED

#include "defs.h"
#include "main.h"


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



void    USART_Config(uint8_t ucPORT, uint32_t ulBaudRate, uint32_t ulDataBits,  uint32_t ulParity );
void    USART_SendString(const char* str);
uint8_t GetIndexByBaudrate(uint32_t baudrate);

#endif /* USART_H_INCLUDED */

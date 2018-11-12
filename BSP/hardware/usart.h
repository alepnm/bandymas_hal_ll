#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED

#include "defs.h"
#include "main.h"


void    USART_Config(uint8_t ucPORT, uint32_t ulBaudRate, uint32_t ulDataBits,  uint32_t ulParity );
void    USART_SendString(const char* str);
uint8_t GetIndexByBaudrate(uint32_t baudrate);

#endif /* USART_H_INCLUDED */

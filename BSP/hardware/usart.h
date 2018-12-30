#ifndef USART_H_INCLUDED
#define USART_H_INCLUDED

#include "defs.h"

enum { BR2400 = 0, BR4800, BR9600, BR19200, BR38400, BR57600 };
enum { USART_STATE_IDLE = 0, USART_STATE_RX, USART_STATE_TX };

extern USART_TypeDef* ports[];
extern const uint32_t baudrates[];


void    USART_Config(uint8_t ucPORT, uint32_t ulBaudRate, uint32_t ulDataBits,  uint8_t ulParity);
void    USART_SendByte(uint8_t data);
void    USART_IRQ_Handler(void);

void    USART_Send(void* buf, size_t size_of_data);
void    USART_SendString(const char* str);
uint8_t GetIndexByBaudrate(uint32_t baudrate);

#endif /* USART_H_INCLUDED */

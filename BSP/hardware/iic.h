#ifndef IIC_H_INCLUDED
#define IIC_H_INCLUDED

#include "defs.h"

uint8_t     IIC_Check(uint8_t iic_addr);
uint8_t     IIC_Write(uint8_t iic_addr, uint16_t reg, uint8_t *buf, uint8_t len);
uint8_t     IIC_Read(uint8_t iic_addr, uint16_t reg, uint8_t *buf, uint8_t len);
uint8_t     IIC_ReadByte(uint16_t reg);
void        IIC_WriteByte(uint16_t reg, uint8_t data);

#endif /* IIC_H_INCLUDED */

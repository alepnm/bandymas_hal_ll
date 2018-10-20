#ifndef HARDWARE_H_INCLUDED
#define HARDWARE_H_INCLUDED

#include "defs.h"
#include "main.h"

#define ENTER_CRITICAL_SECTION() {uint32_t flag; flag = __get_PRIMASK();
#define EXIT_CRITICAL_SECTION()  __set_PRIMASK(flag);}


void        Delay_ms(uint32_t delay);

void        SPI_Transmit8(uint8_t* txdata, uint16_t len);
void        SPI_Receive8(uint8_t* rxdata, uint16_t len);
void        SPI_TransmitReceive8(uint8_t* txdata, uint8_t* rxdata, uint16_t len);

uint16_t    ADC_StartConversion(uint32_t channel, uint32_t resolution);

uint8_t     IIC_Check(uint8_t iic_addr);
uint8_t     IIC_Write(uint8_t iic_addr, uint16_t reg, uint16_t len, uint8_t *buf);
uint8_t     IIC_Read(uint8_t iic_addr, uint16_t reg, uint16_t len, uint8_t *buf);

uint8_t     IIC_ReadByteInst(uint16_t reg);
void        IIC_WriteByteInst(uint16_t reg, uint8_t data);

uint8_t     IIC_ReadByte(uint16_t reg);
uint16_t    IIC_ReadWord(uint16_t reg);
uint32_t    IIC_ReadDWord(uint16_t reg);
void        IIC_WriteByte(uint16_t reg, uint8_t val);
void        IIC_WriteWord(uint16_t reg, uint16_t val);
void        IIC_WriteDWord(uint16_t reg, uint32_t val);



#endif /* HARDWARE_H_INCLUDED */

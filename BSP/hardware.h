#ifndef HARDWARE_H_INCLUDED
#define HARDWARE_H_INCLUDED

#include "defs.h"
#include "main.h"

#define ENTER_CRITICAL_SECTION() {uint32_t flag; flag = __get_PRIMASK();
#define EXIT_CRITICAL_SECTION()  __set_PRIMASK(flag);}


void SPI_Transmit8(uint8_t* txdata, uint16_t len);
void SPI_Receive8(uint8_t* rxdata, uint16_t len);
void SPI_TransmitReceive8(uint8_t* txdata, uint8_t* rxdata, uint16_t len);


uint16_t ADC_StartConversion(uint32_t channel, uint32_t resolution);


void IIC_Start(void);
void IIC_Process( uint16_t mem_addr, void *data, uint16_t len, uint8_t proc );
uint8_t IIC_Read(uint8_t Addr, uint8_t *data, uint8_t reg, uint8_t len);

uint8_t I2C_read_brekeke(I2C_TypeDef *I2Cx, unsigned char Address, uint8_t reg, int nBytes, unsigned char *data);

uint8_t I2C_HW_Write(uint8_t Addr, uint16_t Reg, uint8_t Value);
uint8_t I2C_HW_Read(uint8_t Addr, uint16_t Reg, uint8_t *Value);


#endif /* HARDWARE_H_INCLUDED */

#ifndef HARDWARE_H_INCLUDED
#define HARDWARE_H_INCLUDED

#include "stm32_assert.h"
#include "defs.h"
#include "main.h"

#define     __enter_critical() {uint32_t irq; irq = __get_PRIMASK();
#define     __exit_critical() __set_PRIMASK(irq);}
#define     ATOMIC_SECTION(X) __enter_critical(); {X}; __exit_critical();

#define     ENTER_CRITICAL_SECTION() {uint32_t flag; flag = __get_PRIMASK();
#define     EXIT_CRITICAL_SECTION()  __set_PRIMASK(flag);}

#define     I2C_ADDR_NACK 1
#define     I2C_OK 0

#define     BEEPER_LEVEL_MSK        0x00007000
#define     BEEPER_TONE_MSK         0x00000700
#define     BEEPER_COUNT_MSK        0x000000FF
#define     SET_BEEPER_LEVEL(x)     Beeper.DataReg ^= (Beeper.DataReg & BEEPER_LEVEL_MSK); Beeper.DataReg |= (x<<12)
#define     GET_BEEPER_LEVEL()      (Beeper.DataReg & BEEPER_LEVEL_MSK)>>12
#define     SET_BEEPER_TONE(x)      Beeper.DataReg ^= (Beeper.DataReg & BEEPER_TONE_MSK); Beeper.DataReg |= (x<<8)
#define     GET_BEEPER_TONE()       (Beeper.DataReg & BEEPER_TONE_MSK)>>8
#define     SET_BEEPER_COUNTER(x)   Beeper.DataReg ^= (Beeper.DataReg & BEEPER_COUNT_MSK); Beeper.DataReg |= x;
#define     GET_BEEPER_COUNTER()    (Beeper.DataReg & BEEPER_COUNT_MSK)

/* Beeper */
struct _beeper{
    uint32_t    DataReg;
}Beeper;

extern struct _beeper Beeper;


void        Delay_ms(uint32_t delay);

void        HW_Init(void);
void        USART_Config(uint8_t ucPORT, uint32_t ulBaudRate, uint32_t ulDataBits,  uint32_t ulParity );

void        SPI_Transmit8(uint8_t* txdata, uint16_t len);
void        SPI_Receive8(uint8_t* rxdata, uint16_t len);
void        SPI_TransmitReceive8(uint8_t* txdata, uint8_t* rxdata, uint16_t len);

uint16_t    ADC_StartConversion(uint32_t channel, uint32_t resolution);

uint8_t     IIC_Init(void);
uint8_t     IIC_Check(uint8_t iic_addr);
uint8_t     IIC_Write(uint8_t iic_addr, uint16_t reg, uint8_t *buf, uint16_t len);
uint8_t     IIC_Read(uint8_t iic_addr, uint16_t reg, uint8_t *buf, uint16_t len);

uint8_t     IIC_ReadByteInst(uint16_t reg);
void        IIC_WriteByteInst(uint16_t reg, uint8_t data);

#endif /* HARDWARE_H_INCLUDED */

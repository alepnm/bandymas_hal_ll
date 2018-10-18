#ifndef NEXTION_H_INCLUDED
#define NEXTION_H_INCLUDED


#define LCD_CS_LOW()        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6)
#define LCD_CS_HIGH()       LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_6)
#define LCD_RST_LOW()       LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_7)
#define LCD_RST_HIGH()      LL_GPIO_SetOutputPin(GPIOF, LL_GPIO_PIN_7)
#define LCD_RS_LOW()        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9)
#define LCD_RS_HIGH()       LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9)
#define LCD_RD_LOW()        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define LCD_RD_HIGH()       LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define LCD_WR_LOW()        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12)
#define LCD_WR_HIGH()       LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_12)

#define PULSE_WR_LOW()      LCD_WR_LOW(); asm("nop"); LCD_WR_HIGH();


/* Touch Panel */
#define XPT_CS_LOW()        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2)
#define XPT_CS_HIGH()       LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2)
#define XPT_CLK_LOW()       LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3)
#define XPT_CLK_HIGH()      LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3)

/* SPI-Flash */
#define W25Q_CS_LOW()       asm("nop"); LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4) asm("nop");
#define W25Q_CS_HIGH()      asm("nop"); LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4) asm("nop");

/* Micro SD Card */
#define SD_CS_LOW()         LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5)
#define SD_CS_HIGH()    	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5)




#endif /* NEXTION_H_INCLUDED */

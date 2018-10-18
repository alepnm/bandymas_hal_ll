#ifndef STP_H_INCLUDED
#define STP_H_INCLUDED

#include "hardware.h"
#include "stm32_assert.h"

#define __enter_critical() {uint32_t irq; irq = __get_PRIMASK();
#define __exit_critical() __set_PRIMASK(irq);}
#define ATOMIC_SECTION(X) __enter_critical(); {X}; __exit_critical();

//#define ENTER_CRITICAL_SECTION() {uint32_t flag; flag = __get_PRIMASK();
//#define EXIT_CRITICAL_SECTION()  __set_PRIMASK(flag);}

#define LEDS_ON()               LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14|LL_GPIO_PIN_15)
#define LEDS_OFF()              LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14|LL_GPIO_PIN_15)
#define LEDS_TOGGLE()           LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_14|LL_GPIO_PIN_15)

#define STATUS_LED_ON()         LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15)
#define STATUS_LED_OFF()        LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15)
#define STATUS_LED_TOGGLE()     LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_15)

#define ERROR_LED_ON()          LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14)
#define ERROR_LED_OFF()         LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14)
#define ERROR_LED_TOGGLE()      LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_14)

#define ALARM_RELAY_ON()        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13)
#define ALARM_RELAY_OFF()       LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13)

#define L6470_CS_LOW()          LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11)
#define L6470_CS_HIGH()         LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11)
#define L6470_RST_LOW()         LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10)
#define L6470_RST_HIGH()        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10)

#define M25AA_CS_LOW()          LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12)
#define M25AA_CS_HIGH()         LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12)

#define HC598_CS_LOW()          LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13)
#define HC598_CS_HIGH()         LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13)
#define HC598_LATCH_LOW()       LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_6)
#define HC598_LATCH_HIGH()      LL_GPIO_SetOutputPin(GPIOF, LL_GPIO_PIN_6)
#define HC598_CTRL_LOW()        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define HC598_CTRL_HIGH()       LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11)

#define READ_DI0_INPUT()        LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_0)
#define READ_DI1_INPUT()        LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_1)
#define READ_DI2_INPUT()        LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_2)
#define READ_DI3_INPUT()        LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_14)


#define READ_REFINT()           BSP_ReadADC(LL_ADC_CHANNEL_VREFINT, LL_ADC_RESOLUTION_12B);
#define READ_VBUS()             BSP_ReadADC(LL_ADC_CHANNEL_0, LL_ADC_RESOLUTION_8B);
#define READ_SPREQ()            BSP_ReadADC(LL_ADC_CHANNEL_1, LL_ADC_RESOLUTION_8B);
#define READ_MCUTEMP()          BSP_ReadADC(LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_RESOLUTION_12B);




#define                         BEEPER_LEVEL_MSK    0x00007000
#define                         BEEPER_TONE_MSK     0x00000700
#define                         BEEPER_COUNT_MSK    0x000000FF
#define                         SET_BEEPER_LEVEL(x)     Beeper.DataReg ^= (Beeper.DataReg & BEEPER_LEVEL_MSK); Beeper.DataReg |= (x<<12)
#define                         GET_BEEPER_LEVEL()      (Beeper.DataReg & BEEPER_LEVEL_MSK)>>12
#define                         SET_BEEPER_TONE(x)      Beeper.DataReg ^= (Beeper.DataReg & BEEPER_TONE_MSK); Beeper.DataReg |= (x<<8)
#define                         GET_BEEPER_TONE()       (Beeper.DataReg & BEEPER_TONE_MSK)>>8
#define                         SET_BEEPER_COUNTER(x)   Beeper.DataReg ^= (Beeper.DataReg & BEEPER_COUNT_MSK); Beeper.DataReg |= x;
#define                         GET_BEEPER_COUNTER()    (Beeper.DataReg & BEEPER_COUNT_MSK)


#define UNIT_GROUP              0x04
#define UNIT_SUBGROUP           0x01
#define UNIT_NAME               "STP"
#define UNIT_FW_VERSION         "20"
#define UNIT_HW_VERSION         "30"
#define UNIT_PROD_CODE          "GRG060"








/* Private functions */
void        BSP_Start(void);
void        BSP_UartConfig(uint8_t ucPORT, uint32_t ulBaudRate, uint8_t ucDataBits,  uint8_t eParity );
void        BSP_UartSendString(const char* str);

void        BSP_ReadADC(uint32_t channel, uint32_t resolution);

void        BSP_ReadDipSwitch(void);

void        BSP_Delay(uint32_t delay);
uint8_t     CheckBaudrateValue(uint32_t baudrate);
uint8_t     CheckBaudrateIndex(uint8_t idx);
uint8_t     GetIndexByBaudrate(uint32_t baudrate);
uint32_t    GetBaudrateByIndex(uint8_t idx);
uint8_t     GetCurrentBaudrateIndex(void);

#endif /* STP_H_INCLUDED */

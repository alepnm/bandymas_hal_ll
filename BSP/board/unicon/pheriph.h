#ifndef PHERIPH_H_INCLUDED
#define PHERIPH_H_INCLUDED

#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_spi.h"
#include "stm32f0xx_ll_rtc.h"


void UNI_TimerStart(TIM_TypeDef* TIMx);
void UNI_TimerStop(TIM_TypeDef* TIMx);
void UNI_TimerEnableChannel(TIM_TypeDef* TIMx, uint32_t Channel);
void UNI_TimerDisableChannel(TIM_TypeDef* TIMx, uint32_t Channel);
void UNI_PWM_SetDuty(uint8_t duty);


#endif /* PHERIPH_H_INCLUDED */

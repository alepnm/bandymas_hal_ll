#ifndef HARDWARE_H_INCLUDED
#define HARDWARE_H_INCLUDED

#include "stm32_assert.h"
#include "usart.h"
#include "spi.h"
#include "iic.h"
#include "adc.h"

#define     __enter_critical() {uint32_t irq; irq = __get_PRIMASK();
#define     __exit_critical() __set_PRIMASK(irq);}
#define     ATOMIC_SECTION(X) __enter_critical(); {X}; __exit_critical();

#define     ENTER_CRITICAL_SECTION() {uint32_t flag; flag = __get_PRIMASK();
#define     EXIT_CRITICAL_SECTION()  __set_PRIMASK(flag);}

void        HW_Init(void);

#endif /* HARDWARE_H_INCLUDED */

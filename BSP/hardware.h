#ifndef HARDWARE_H_INCLUDED
#define HARDWARE_H_INCLUDED

#include "stm32_assert.h"
#include "main.h"
#include "defs.h"

#define     __enter_critical() {uint32_t irq; irq = __get_PRIMASK();
#define     __exit_critical() __set_PRIMASK(irq);}
#define     ATOMIC_SECTION(X) __enter_critical(); {X}; __exit_critical();

#define     ENTER_CRITICAL_SECTION() __enter_critical()
#define     EXIT_CRITICAL_SECTION()  __exit_critical()

void        HW_Init(void);

#endif /* HARDWARE_H_INCLUDED */

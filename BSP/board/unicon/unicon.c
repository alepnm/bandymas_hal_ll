

#include "unicon.h"


void UNI_Start(void) {

    SysTick_Config(SystemCoreClock/1000);

    /* startuojam */
    LEDS_OFF();
}



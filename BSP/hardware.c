

#include "hardware.h"
#include "adc.h"


/*   */
void HW_Init(void){

    SysTick_Config(SystemCoreClock/1000);

    ADC_Init();



}


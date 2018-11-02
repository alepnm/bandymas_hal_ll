#include "main.h"

extern volatile uint32_t timestamp;

/*  */
void SystickDelay_ms(uint32_t delay) {

    delay += timestamp;

    while(delay > timestamp);
}


/*  */
void Delay_ms(uint32_t delay) {

    LL_RCC_ClocksTypeDef RCC_Clocks;

    LL_RCC_GetSystemClocksFreq(&RCC_Clocks);

    uint32_t nCount = (RCC_Clocks.SYSCLK_Frequency/14000)*delay;

    while(nCount-- > 0);
}

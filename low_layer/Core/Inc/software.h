#ifndef SOFTWARE_H_INCLUDED
#define SOFTWARE_H_INCLUDED

#include "defs.h"

#define     BEEPER_LEVEL_MSK        0x00007000
#define     BEEPER_TONE_MSK         0x00000700
#define     BEEPER_COUNT_MSK        0x000000FF
#define     SET_BEEPER_LEVEL(x)     Beeper.DataReg ^= (Beeper.DataReg & BEEPER_LEVEL_MSK); Beeper.DataReg |= (x<<12)
#define     GET_BEEPER_LEVEL()      (Beeper.DataReg & BEEPER_LEVEL_MSK)>>12
#define     SET_BEEPER_TONE(x)      Beeper.DataReg ^= (Beeper.DataReg & BEEPER_TONE_MSK); Beeper.DataReg |= (x<<8)
#define     GET_BEEPER_TONE()       (Beeper.DataReg & BEEPER_TONE_MSK)>>8
#define     SET_BEEPER_COUNTER(x)   Beeper.DataReg ^= (Beeper.DataReg & BEEPER_COUNT_MSK); Beeper.DataReg |= x
#define     GET_BEEPER_COUNTER()    (Beeper.DataReg & BEEPER_COUNT_MSK)

/* Beeper */
struct _beeper{
    uint32_t    DataReg;
}Beeper;

extern struct _beeper Beeper;


void SW_SystemStart(void);
void SW_SystemExecute(void);
void SW_SystemDataUpdate(void);
void SW_ModbusDataUpdate(void);

void SystickDelay_ms(uint32_t delay);
void Delay_ms(uint32_t delay);

#endif /* SOFTWARE_H_INCLUDED */

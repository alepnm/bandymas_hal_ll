#ifndef ADC_H_INCLUDED
#define ADC_H_INCLUDED

typedef struct{
    uint32_t    AdcVal;
    uint32_t    ConvertedValue;
    uint32_t    Resolution;
}ADC_IntRegister_TypeDef;

typedef struct {
   ADC_IntRegister_TypeDef VRefInt;
   ADC_IntRegister_TypeDef McuTemp;
   ADC_IntRegister_TypeDef VBat;
}ADC_IntRegs_TypeDef;

extern ADC_IntRegs_TypeDef ADC_InternalRegisters;




void        ADC_Init(void);

void        ADC_Read_VREFINT(void);
void        ADC_Read_MCUTEMP(void);
void        ADC_Read_VBAT(void);

uint16_t    ADC_StartConversion(uint32_t channel, uint32_t resolution);
uint16_t    ADC_ConvertToMvolts(uint16_t adcval, uint32_t resolution);


#endif /* ADC_H_INCLUDED */

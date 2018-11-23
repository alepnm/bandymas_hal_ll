#include "main.h"
#include "adc.h"


ADC_IntRegs_TypeDef ADC_InternalRegisters;

/*  */
void ADC_Init(void){

    ADC_InternalRegisters.VRefInt.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InternalRegisters.McuTemp.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InternalRegisters.VBat.Resolution = LL_ADC_RESOLUTION_10B;

    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));

    LL_ADC_ClearFlag_ADRDY(ADC1);

    ADC_Read_VREFINT();
    ADC_Read_MCUTEMP();
    ADC_Read_VBAT();
}


/*  */
uint16_t ADC_StartConversion(uint32_t channel, uint32_t resolution) {

    uint16_t result;

    ADC1->CHSELR = channel;
    LL_ADC_SetResolution(ADC1, resolution);

    do {
        LL_ADC_Enable(ADC1);
    } while( !LL_ADC_IsActiveFlag_ADRDY(ADC1) );

    LL_ADC_REG_StartConversion(ADC1);
    while( !LL_ADC_IsActiveFlag_EOC(ADC1) );

    switch(resolution) {
    case LL_ADC_RESOLUTION_6B:
        result = LL_ADC_REG_ReadConversionData6(ADC1);
        break;
    case LL_ADC_RESOLUTION_8B:
        result = LL_ADC_REG_ReadConversionData8(ADC1);
        break;
    case LL_ADC_RESOLUTION_10B:
        result = LL_ADC_REG_ReadConversionData10(ADC1);
        break;
    case LL_ADC_RESOLUTION_12B:
        result = LL_ADC_REG_ReadConversionData12(ADC1);
        break;
    }

    do {
        LL_ADC_Disable(ADC1);
    } while( LL_ADC_IsEnabled(ADC1) );

    //Delay_ms(1);

    return result;
}


/* Skaitom VREF reiksme */
void ADC_Read_VREFINT(void){
    ADC_InternalRegisters.VRefInt.AdcVal = ADC_StartConversion(LL_ADC_CHANNEL_VREFINT, ADC_InternalRegisters.VRefInt.Resolution);
    ADC_InternalRegisters.VRefInt.ConvertedValue = __LL_ADC_CALC_VREFANALOG_VOLTAGE( ADC_InternalRegisters.VRefInt.AdcVal, LL_ADC_RESOLUTION_12B );
}


/* Skaitom MCUTEMP reiksme */
void ADC_Read_MCUTEMP(void){
    ADC_InternalRegisters.McuTemp.AdcVal = ADC_StartConversion(LL_ADC_CHANNEL_TEMPSENSOR, ADC_InternalRegisters.McuTemp.Resolution);
    ADC_InternalRegisters.McuTemp.ConvertedValue = __LL_ADC_CALC_TEMPERATURE( ADC_InternalRegisters.VRefInt.ConvertedValue, ADC_InternalRegisters.McuTemp.AdcVal, LL_ADC_RESOLUTION_12B );
}


/*  */
void ADC_Read_VBAT(void){
    ADC_InternalRegisters.VBat.AdcVal = ADC_StartConversion(LL_ADC_CHANNEL_VBAT, ADC_InternalRegisters.VBat.Resolution);
    ADC_InternalRegisters.VBat.ConvertedValue = __LL_ADC_CALC_DATA_TO_VOLTAGE( ADC_InternalRegisters.VRefInt.ConvertedValue, ADC_InternalRegisters.VBat.AdcVal, LL_ADC_RESOLUTION_10B ) * 2;
}


/*  */
uint16_t ADC_ConvertToMvolts(uint16_t adcval, uint32_t resolution){
    return __LL_ADC_CALC_DATA_TO_VOLTAGE(ADC_InternalRegisters.VRefInt.ConvertedValue, adcval, resolution );
}

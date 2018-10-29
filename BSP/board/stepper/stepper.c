

#include "stepper.h"
#include "user_mb_app.h"
#include "M25AAxx.h"
#include "l6470.h"


extern LL_USART_InitTypeDef USART_InitStruct;
extern USART_TypeDef* ports[3u];
extern const uint32_t baudrates[6u];

SmcHandle_TypeDef SMC_Control;

void STP_Start(void) {

    uint8_t i = 0;

    SysTick_Config(SystemCoreClock/1000);

    LEDS_OFF();

    ALARM_RELAY_OFF();

    L6470_CS_HIGH();
    L6470_RST_HIGH();
    M25AA_CS_HIGH();
    HC598_CS_HIGH();
    HC598_LATCH_HIGH();

    /* ADC paruosimas */
    LL_ADC_StartCalibration(ADC1);
    while( LL_ADC_IsCalibrationOnGoing(ADC1) != 0 );


#if defined(MODBUS_ENABLE)
    /* conntrol TIM6 */
    LL_TIM_DisableCounter(TIM6);
    LL_TIM_SetCounter(TIM6, 0);
#endif

    /* control TIM16 */
    LL_TIM_OC_SetCompareCH1(TIM16, 0);
    LL_TIM_EnableCounter(TIM16);
    LL_TIM_EnableAllOutputs(TIM16); //
    LL_TIM_CC_DisableChannel(TIM16, LL_TIM_CHANNEL_CH1);


    /* control TIM17 */
    LL_TIM_OC_SetCompareCH1(TIM17, 0);
    LL_TIM_EnableCounter(TIM17);
    LL_TIM_EnableAllOutputs(TIM17);
    LL_TIM_CC_DisableChannel(TIM17, LL_TIM_CHANNEL_CH1);


    /* control USART1 */
    /* jai modbusas neaktyvuotas, paliekam pradine konfiguracija ir nustatom patys interaptus */
#if !defined(MODBUS_ENABLE)
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_EnableIT_TC(USART1);
    LL_USART_Enable(USART1);
#endif

    do{
        LL_SPI_Enable(SPI1);
    } while( !LL_SPI_IsEnabled(SPI1) );

    STP_ReadDipSwitch();
    M25AAxx_Init();


    SET_BEEPER_TONE(0);
    SET_BEEPER_LEVEL(10);
    SET_BEEPER_COUNTER(25);


    /* konfiguruojam Modbus */
#if defined(MODBUS_ENABLE)
    if( eMBInit( MB_RTU, (UCHAR)(*MbPortParams.MbAddr.pmbus), MbPortParams.Uart, (ULONG)( GetBaudrateByIndex(*MbPortParams.Baudrate.pmbus) ), (eMBParity)(*MbPortParams.Parity.pmbus) ) == MB_ENOERR ){
        if( eMBEnable() == MB_ENOERR ){
            if( eMBSetSlaveID( 123, TRUE, ucSlaveIdBuf, (MB_FUNC_OTHER_REP_SLAVEID_BUF - 4) ) == MB_ENOERR ){
                MbPortParams.ModbusActive = true;
            }
        }
    }
#endif

    SMC_Control.StrData.pHWVersion = ucSlaveIdBuf + 3;
    SMC_Control.StrData.pFWVersion = ucSlaveIdBuf + 6;
    SMC_Control.StrData.pId = ucSlaveIdBuf + 12;
    SMC_Control.StrData.pName = ucSlaveIdBuf + 19;
    SMC_Control.StrData.pProdCode = ucSlaveIdBuf + 23;


    ucSlaveIdBuf[0] =   UNIT_GROUP;
    ucSlaveIdBuf[1] =   UNIT_SUBGROUP;
    ucSlaveIdBuf[2] =   'H';
    ucSlaveIdBuf[5] =   'F';
    ucSlaveIdBuf[8] =   'S';
    ucSlaveIdBuf[11] =  'I';
    ucSlaveIdBuf[18] =  'e';

    memcpy(SMC_Control.StrData.pFWVersion, UNIT_FW_VERSION, 2);
    memcpy(SMC_Control.StrData.pHWVersion, UNIT_HW_VERSION, 2);
    memcpy(SMC_Control.StrData.pName, UNIT_NAME, 3);
    memcpy(SMC_Control.StrData.pProdCode, UNIT_PROD_CODE, 7);


    do{
        *(SMC_Control.StrData.pId + i) = M25AAxx.UidBuffer[i];
    }while(++i < M25AAxx_UID_BUFFER_SIZE);

}


/*  */
void STP_UartSendString(const char* str){

    uint8_t i = 0;

    while(*str > 0x00){

        LL_USART_TransmitData8(USART1, *str+i);
        i++;
    }
}



/*  */
void STP_SystemDataUpdate(void){

    /*  */
    STP_ReadDipSwitch();

    /*  */
    READ_REFINT();
    READ_VBUS();
    READ_SPREQ();
    READ_MCUTEMP();

}


/*   */
void STP_ModbusDataUpdate(void){


    usRegInputBuf[IR_SPREQ_VALUE] = SMC_Control.ADC_Vals.SpReq.mV;
    usRegInputBuf[IR_MCUTEMP] = SMC_Control.ADC_Vals.McuTemp.celsius;

    xMbSetDInput( DI_DI0_STATE, READ_DI0_INPUT() );
    xMbSetDInput( DI_DI1_STATE, READ_DI1_INPUT() );
    xMbSetDInput( DI_DI2_STATE, READ_DI2_INPUT() );
    xMbSetDInput( DI_DI3_STATE, READ_DI3_INPUT() );

}



/*  */
void STP_ReadADC(uint32_t channel, uint32_t resolution) {

    uint16_t* dest;
    uint16_t vref_mv = SMC_Control.ADC_Vals.VRef.mV;

    ADC1->CHSELR = channel;
    LL_ADC_SetResolution(ADC1, resolution);

    LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_55CYCLES_5);

    switch(channel) {
    case LL_ADC_CHANNEL_0:
        dest = &SMC_Control.ADC_Vals.Vbus.adc;
        break;
    case LL_ADC_CHANNEL_1:
        dest = &SMC_Control.ADC_Vals.SpReq.adc;
        break;
    case LL_ADC_CHANNEL_VREFINT:
        dest = &SMC_Control.ADC_Vals.VRef.adc;
        LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);
        break;
    case LL_ADC_CHANNEL_TEMPSENSOR:
        dest = &SMC_Control.ADC_Vals.McuTemp.adc;
        break;
    }

    *dest = ADC_StartConversion(channel, resolution);

    switch(channel) {
    case LL_ADC_CHANNEL_0:
        SMC_Control.ADC_Vals.Vbus.mV = ( __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mv, *dest, resolution) ) * 8;
        break;
    case LL_ADC_CHANNEL_1:
        SMC_Control.ADC_Vals.SpReq.mV = ( __LL_ADC_CALC_DATA_TO_VOLTAGE(vref_mv, *dest, resolution) ) * 3.2;
        break;
    case LL_ADC_CHANNEL_VREFINT:
        SMC_Control.ADC_Vals.VRef.mV = __LL_ADC_CALC_VREFANALOG_VOLTAGE( *dest, resolution);
        break;
    case LL_ADC_CHANNEL_TEMPSENSOR:
        SMC_Control.ADC_Vals.McuTemp.celsius = __LL_ADC_CALC_TEMPERATURE(vref_mv, *dest, resolution);
        break;
    }
}



/*  */
void STP_ReadDipSwitch(void) {

    uint8_t delay = 10, dipsw = 0;
    static uint8_t ldipsw = 0;

    HC598_LATCH_LOW();
    while(delay--);
    delay = 10;
    HC598_LATCH_HIGH();

    HC598_CTRL_LOW();
    while(delay--);
    delay = 10;
    HC598_CTRL_HIGH();

    HC598_CS_LOW();
    SPI_Receive8(&SMC_Control.DipSwitch.Data, 1);
    HC598_CS_HIGH();

    if( SMC_Control.DipSwitch.Data != ldipsw ){
        SET_BEEPER_COUNTER(25);
        ldipsw = SMC_Control.DipSwitch.Data;
    }

    dipsw = SMC_Control.DipSwitch.Data^0xFF;

    /* jai esam STOP rezime nustatom parametrus */
    SMC_Control.DipSwitch.Option.MotorType = ( dipsw & 0x07 );
    SMC_Control.DipSwitch.Option.Scrolling = ( dipsw>>4 & 0x01 );
    SMC_Control.DipSwitch.Option.HallSensor = ( dipsw>>5 & 0x01 );
    SMC_Control.DipSwitch.Option.ControlMode = ( dipsw>>6 & 0x03 );

    /* issaugojam nuskaitytus parametrus modbus registre */
    usRegInputBuf[IR_MAX_RPM] = ( READ_BIT(dipsw, 0x01<<3) == FALSE ) ? 150 : 200;
    xMbSetDInput( DI_SW1_STATE, SMC_Control.DipSwitch.Data & 0x01 );
    xMbSetDInput( DI_SW2_STATE, SMC_Control.DipSwitch.Data>>1 & 0x01 );
    xMbSetDInput( DI_SW3_STATE, SMC_Control.DipSwitch.Data>>2 & 0x01 );
    xMbSetDInput( DI_SW4_STATE, SMC_Control.DipSwitch.Data>>3 & 0x01 );
    xMbSetDInput( DI_SW5_STATE, SMC_Control.DipSwitch.Data>>4 & 0x01 );
    xMbSetDInput( DI_SW6_STATE, SMC_Control.DipSwitch.Data>>5 & 0x01 );
    xMbSetDInput( DI_SW7_STATE, SMC_Control.DipSwitch.Data>>6 & 0x01 );
    xMbSetDInput( DI_SW8_STATE, SMC_Control.DipSwitch.Data>>7 & 0x01 );
}



/* chekinam bodreito reiksme - ar standartine? */
uint8_t CheckBaudrateValue(uint32_t baudrate) {

    if( GetIndexByBaudrate( baudrate ) == 0xFF ) return 1;

    return 0;
}


uint8_t CheckBaudrateIndex( uint8_t idx ) {

    if( GetBaudrateByIndex( idx ) == 0xFFFFFFFF ) return 1;

    return 0;
}


/* grazinam bodreito indeksa lenteleje. Jai bodreito reiksme nestandartine grazinam 0xFF */
uint8_t GetIndexByBaudrate( uint32_t baudrate ) {

    uint8_t i = 0;

    while(baudrate != baudrates[i]) {
        if( i >= ( sizeof(baudrates)/sizeof(baudrate) ) ) {
            i = 0xFF;
            break;
        }
        i++;
    }
    return i;
}


/* grazinam bodreita pagal jo indeksa lenteleje. Jai indeksas didesnis uz standartiniu bodreitu skaiciu,
grazinam 0x00 */
uint32_t GetBaudrateByIndex( uint8_t idx ) {

    if( idx > sizeof(baudrates)/sizeof(uint32_t) ) return 0x00;

    return baudrates[idx];
}


/*  */
uint8_t GetCurrentBaudrateIndex( void ) {
    return GetIndexByBaudrate( USART_InitStruct.BaudRate );
}




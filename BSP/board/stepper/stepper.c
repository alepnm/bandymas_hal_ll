

#include "stepper.h"
#include "hardware.h"
#include "user_mb_app.h"
#include "M25AAxx.h"



USART_TypeDef* ports[] = {USART1, NULL, NULL};

extern LL_USART_InitTypeDef USART_InitStruct;
extern M25AAxx_TypeDef M25AAxx;

static const uint32_t baudrates[6u] = {2400u, 4800u, 9600u, 19200u, 38400u, 57600u};

uint8_t iic_buf[100];

void BSP_Start(void) {

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

    BSP_ReadDipSwitch();
    M25AAxx_Init();


    SET_BEEPER_TONE(0);
    SET_BEEPER_LEVEL(10);
    SET_BEEPER_COUNTER(25);



    LL_I2C_SetMasterAddressingMode(I2C1, LL_I2C_ADDRESSING_MODE_7BIT);

    do{
        LL_I2C_Enable(I2C1);
    } while( LL_I2C_IsEnabled(I2C1) == RESET );

    while( LL_I2C_IsActiveFlag_BUSY(I2C1) );

    iic_buf[0] = 0x55;

    //IIC_Write(0xA0, 0, 0x12);
    IIC_Read(0xA0, 0, 100, iic_buf);

    uint16_t wrd = IIC_ReadWord(0);


     do{
        LL_I2C_Disable(I2C1);
    } while( LL_I2C_IsEnabled(I2C1) == SET );



}


/*  */
void BSP_UartConfig(uint8_t ucPORT, uint32_t ulBaudRate, uint8_t ucDataBits,  uint8_t eParity ) {

    do{
         LL_USART_Disable(ports[ucPORT]);
    }while( LL_USART_IsEnabled(ports[ucPORT]) );

    if( CheckBaudrateValue( ulBaudRate ) ) {
        ulBaudRate = GetBaudrateByIndex( MBBAURATE_DEF );
        usRegHoldingBuf[HR_MBBAUDRATE] = MBBAURATE_DEF;
    }

    USART_InitStruct.BaudRate = ulBaudRate;

    switch(eParity) {
    case MB_PAR_ODD:
        USART_InitStruct.Parity = LL_USART_PARITY_ODD;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
        break;
    case MB_PAR_EVEN:
        USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
        break;
    default:
        USART_InitStruct.Parity = LL_USART_PARITY_NONE;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    }

    LL_USART_Init(ports[ucPORT], &USART_InitStruct);

    do{
        LL_USART_Enable(ports[ucPORT]);
    }while( !LL_USART_IsEnabled(ports[ucPORT]) );
}


/*  */
void BSP_UartSendString(const char* str){

    uint8_t i = 0;

    while(*str > 0x00){

        LL_USART_TransmitData8(USART1, *str+i);
        i++;
    }
}


/*  */
void BSP_ReadADC(uint32_t channel, uint32_t resolution) {

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
void BSP_ReadDipSwitch(void) {

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




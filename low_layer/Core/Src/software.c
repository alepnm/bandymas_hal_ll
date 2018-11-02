#include "software.h"
#include "iic_eeprom.h"
#include "user_mb_app.h"


extern LL_RTC_TimeTypeDef RTC_Time;
extern LL_RTC_DateTypeDef RTC_Date;

static void SW_System_Init(void);
static void SW_EeDataRestore(void);
static void SW_DataInitDefaults(void);

/*  */
void SW_SystemStart(void){

    /* inicializuojam sistemine aplinka: pointerius, pradines reiksmes  */
    SW_System_Init();
}


/*  */
static void SW_System_Init(void){

    /* atstatom EEPROM duomenys */
    SW_EeDataRestore();

}


/*  */
static void SW_EeDataRestore(void){

    /* inicializuojam EEPROM defaultais, jai jis neinicializuotas */
    if(EEP24XX_ReadByte(EEADR_INITBYTE) != EE_INITBYTE_DEF){
        SW_DataInitDefaults();
    }




    EEP24XX_Read( EEADR_MBPORTPARAMS, (uint8_t*)&MbPortParams, sizeof(MbPortParams) );
    EEP24XX_Read( EEADR_RTCTIME, (uint8_t*)&RTC_Time, sizeof(RTC_Time) );
    EEP24XX_Read( EEADR_RTCDATE, (uint8_t*)&RTC_Date, sizeof(RTC_Date) );

    LL_RTC_EnableInitMode(RTC);
    LL_RTC_DisableWriteProtection(RTC);

    LL_RTC_TIME_Config(RTC, RTC_Time.TimeFormat, RTC_Time.Hours, RTC_Time.Minutes, RTC_Time.Seconds);
    LL_RTC_DATE_Config(RTC, RTC_Date.WeekDay, RTC_Date.Day, RTC_Date.Month, RTC_Date.Year);

    LL_RTC_EnableWriteProtection(RTC);
    LL_RTC_DisableInitMode(RTC);
}


/*  */
static void SW_DataInitDefaults(void) {

    usRegHoldingBuf[HR_MBADDR] = MBADDR_DEF;
    usRegHoldingBuf[HR_MBBAUDRATE] = MBBAURATE_DEF;
    usRegHoldingBuf[HR_MBPARITY] = MB_PAR_NONE;
    usRegHoldingBuf[HR_MBSTOPBITS] = MBSTOPBITS_DEF;

    MbPortParams.MbAddr.pmbus = &usRegHoldingBuf[HR_MBADDR];
    MbPortParams.Baudrate.pmbus = &usRegHoldingBuf[HR_MBBAUDRATE];
    MbPortParams.Parity.pmbus = &usRegHoldingBuf[HR_MBPARITY];
    MbPortParams.DataBits.pmbus = NULL;
    MbPortParams.StopBits.pmbus = &usRegHoldingBuf[HR_MBSTOPBITS];

    MbPortParams.Uart = MB_PORT_DEF;
    MbPortParams.MbAddr.cvalue = usRegHoldingBuf[HR_MBADDR];
    MbPortParams.Baudrate.cvalue = usRegHoldingBuf[HR_MBBAUDRATE];
    MbPortParams.Parity.cvalue = usRegHoldingBuf[HR_MBPARITY];
    MbPortParams.DataBits.cvalue = MBWORDLENGHT_DEF;
    MbPortParams.StopBits.cvalue = usRegHoldingBuf[HR_MBSTOPBITS];

    EEP24XX_Write( EEADR_MBPORTPARAMS, (uint8_t*)&MbPortParams, sizeof(MbPortParams) );

    RTC_Time.Seconds = 0;
    RTC_Time.Minutes = 0;
    RTC_Time.Hours = 0;

    EEP24XX_Write( EEADR_RTCTIME, (uint8_t*)&RTC_Time, sizeof(RTC_Time) );

    RTC_Date.WeekDay = LL_RTC_WEEKDAY_MONDAY;
    RTC_Date.Day = 1;
    RTC_Date.Month = LL_RTC_MONTH_JANUARY;
    RTC_Date.Year = 18;

    EEP24XX_Write( EEADR_RTCDATE, (uint8_t*)&RTC_Date, sizeof(RTC_Date) );



    EEP24XX_WriteByte(EEADR_INITBYTE, EE_INITBYTE_DEF);
}


/*  */
void SW_SystemDataUpdate(void){

}


/*  */
void SW_ModbusDataUpdate(void){


}

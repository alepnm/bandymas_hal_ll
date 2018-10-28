#include "software.h"
#include "M25AAxx.h"

static void StpSystemInit(void);
static void EeDataRestore(void);


void SystemStart(void){

    STP_Start();

    /* inicializuojam sistemine aplinka: pointerius, pradines reiksmes  */
    StpSystemInit();


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


}

/*  */
static void StpSystemInit(void){

    uint8_t i = 0;

    usRegHoldingBuf[HR_MBADDR] = 10;
    usRegHoldingBuf[HR_MBBAUDRATE] = 3;
    usRegHoldingBuf[HR_MBPARITY] = MB_PAR_NONE;
    usRegHoldingBuf[HR_MBSTOPBITS] = 1;



    /* atstatom EEPROM duomenys */
    EeDataRestore();

    /* Init pointers */
    MbPortParams.MbAddr.pmbus = &usRegHoldingBuf[HR_MBADDR];
    MbPortParams.Baudrate.pmbus = &usRegHoldingBuf[HR_MBBAUDRATE];
    MbPortParams.Parity.pmbus = &usRegHoldingBuf[HR_MBPARITY];
    MbPortParams.DataBits.pmbus = NULL;
    MbPortParams.StopBits.pmbus = &usRegHoldingBuf[HR_MBSTOPBITS];


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


    MbPortParams.Uart = 0;
    MbPortParams.MbAddr.cvalue = usRegHoldingBuf[HR_MBADDR];
    MbPortParams.Baudrate.cvalue = usRegHoldingBuf[HR_MBBAUDRATE];
    MbPortParams.Parity.cvalue = usRegHoldingBuf[HR_MBPARITY];
    MbPortParams.DataBits.cvalue = MBWORDLENGHT_DEF;
    MbPortParams.StopBits.cvalue = usRegHoldingBuf[HR_MBSTOPBITS];

}

/*  */
static void EeDataRestore(void){


}




/*  */
void SystemDataUpdate(void){

    /*  */
    STP_ReadDipSwitch();

    /*  */
    READ_REFINT();
    READ_VBUS();
    READ_SPREQ();
    READ_MCUTEMP();

}


void ModbusDataUpdate(void){


    usRegInputBuf[IR_SPREQ_VALUE] = SMC_Control.ADC_Vals.SpReq.mV;
    usRegInputBuf[IR_MCUTEMP] = SMC_Control.ADC_Vals.McuTemp.celsius;

    xMbSetDInput( DI_DI0_STATE, READ_DI0_INPUT() );
    xMbSetDInput( DI_DI1_STATE, READ_DI1_INPUT() );
    xMbSetDInput( DI_DI2_STATE, READ_DI2_INPUT() );
    xMbSetDInput( DI_DI3_STATE, READ_DI3_INPUT() );

}

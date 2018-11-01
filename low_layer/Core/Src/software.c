#include "software.h"

static void SW_System_Init(void);
static void SW_EeDataRestore(void);


void SW_SystemStart(void){

    //STP_Start();

    /* inicializuojam sistemine aplinka: pointerius, pradines reiksmes  */
    SW_System_Init();
}

/*  */
static void SW_System_Init(void){

    /* atstatom EEPROM duomenys */
    SW_EeDataRestore();

    usRegHoldingBuf[HR_MBADDR] = 10;
    usRegHoldingBuf[HR_MBBAUDRATE] = 3;
    usRegHoldingBuf[HR_MBPARITY] = MB_PAR_NONE;
    usRegHoldingBuf[HR_MBSTOPBITS] = 1;

    MbPortParams.MbAddr.pmbus = &usRegHoldingBuf[HR_MBADDR];
    MbPortParams.Baudrate.pmbus = &usRegHoldingBuf[HR_MBBAUDRATE];
    MbPortParams.Parity.pmbus = &usRegHoldingBuf[HR_MBPARITY];
    MbPortParams.DataBits.pmbus = NULL;
    MbPortParams.StopBits.pmbus = &usRegHoldingBuf[HR_MBSTOPBITS];

    MbPortParams.Uart = 0;
    MbPortParams.MbAddr.cvalue = usRegHoldingBuf[HR_MBADDR];
    MbPortParams.Baudrate.cvalue = usRegHoldingBuf[HR_MBBAUDRATE];
    MbPortParams.Parity.cvalue = usRegHoldingBuf[HR_MBPARITY];
    MbPortParams.DataBits.cvalue = MBWORDLENGHT_DEF;
    MbPortParams.StopBits.cvalue = usRegHoldingBuf[HR_MBSTOPBITS];

}

/*  */
static void SW_EeDataRestore(void){


}




/*  */
void SW_SystemDataUpdate(void){

}


void SW_ModbusDataUpdate(void){


}

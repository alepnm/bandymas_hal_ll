#include "user_mb_app.h"

/*------------------------Slave mode use these variables----------------------*/

//Slave mode:DiscreteInputs variables
USHORT   usDiscInputStart = DISCRETE_INPUT_START;
#if (DISCRETE_INPUT_NDISCRETES % 8)
UCHAR    ucDiscInputBuf[DISCRETE_INPUT_NDISCRETES/8+1];
#else
UCHAR    ucDiscInputBuf[DISCRETE_INPUT_NDISCRETES/8];
#endif

//Slave mode:Coils variables
USHORT   usCoilStart = COIL_START;
#if (COIL_NCOILS % 8)
UCHAR    ucCoilBuf[COIL_NCOILS/8+1];
#else
UCHAR    ucCoilBuf[COIL_NCOILS/8];
#endif

//Slave mode:InputRegister variables
USHORT   usRegInputStart = REG_INPUT_START;
USHORT   usRegInputBuf[REG_INPUT_NREGS];

//Slave mode:HoldingRegister variables
USHORT   usRegHoldingStart = REG_HOLDING_START;
USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];

/* SlaveID buferis */
uint8_t ucSlaveIdBuf[MB_FUNC_OTHER_REP_SLAVEID_BUF];

/************************ Modbus Callback Functions **************************/
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs ) {

    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
            && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) ) {

        iRegIndex = ( int )( usAddress - usRegInputStart );

        while( usNRegs > 0 ) {
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    } else {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode ) {

    eMBErrorCode    eStatus = MB_ENOERR;

    if( usAddress >= REG_HOLDING_START
            && ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS )  ) {

        uint16_t iRegIndex = ( uint16_t )( usAddress - usRegHoldingStart );

        switch ( eMode ) {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 ) {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:

            /* kelis nuskaitytus registrus perkeliam i pagrindini masyva */
            while( usNRegs > 0 ) {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;

                // gal isvalom ir priemimo buferi iskarto?..

                iRegIndex++;
                usNRegs--;
            }
            break;
        }
    } else {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode ) {

    eMBErrorCode    eStatus = MB_ENOERR;

    uint16_t iNReg =  usNCoils / 8 + 1;

    if( ( usAddress >= usCoilStart ) &&
            ( usAddress + usNCoils <= COIL_START + COIL_NCOILS ) ) {

        uint16_t iRegIndex    = ( uint16_t )( usAddress - usCoilStart ) / 8 ;
        uint16_t iRegBitIndex = ( uint16_t )( usAddress - usCoilStart ) % 8 ;

        switch ( eMode ) {
            /* Pass current coil values to the protocol stack. */
        case MB_REG_READ:
            while( iNReg > 0 ) {
                *pucRegBuffer++ = xMBUtilGetBits(&ucCoilBuf[iRegIndex++] , iRegBitIndex , 8);
                iNReg --;
            }
            pucRegBuffer --;
            usNCoils = usNCoils % 8;
            *pucRegBuffer = *pucRegBuffer <<(8 - usNCoils);
            *pucRegBuffer = *pucRegBuffer >>(8 - usNCoils);
            break;

            /* Update current coil values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while(iNReg > 1) {
                xMBUtilSetBits(&ucCoilBuf[iRegIndex++], iRegBitIndex, 8, *pucRegBuffer++);
                iNReg--;
            }

            usNCoils = usNCoils % 8;

            if (usNCoils != 0) {
                xMBUtilSetBits(&ucCoilBuf[iRegIndex++], iRegBitIndex, usNCoils,
                               *pucRegBuffer++);
            }
            break;
        }
    } else {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete ) {

    eMBErrorCode    eStatus = MB_ENOERR;

    uint16_t iNReg =  usNDiscrete / 8 + 1;

    if( ( usAddress >= usDiscInputStart )
            && ( usAddress + usNDiscrete <= DISCRETE_INPUT_START + DISCRETE_INPUT_NDISCRETES ) ) {

        uint16_t iRegIndex    = ( uint16_t )( usAddress - usDiscInputStart ) / 8 ;
        uint16_t iRegBitIndex = ( uint16_t )( usAddress - usDiscInputStart ) % 8 ;

        while( iNReg > 0 ) {
            *pucRegBuffer++ = xMBUtilGetBits(&ucDiscInputBuf[iRegIndex++] , iRegBitIndex , 8);
            iNReg --;
        }

        pucRegBuffer --;
        usNDiscrete = usNDiscrete % 8;
        *pucRegBuffer = *pucRegBuffer <<(8 - usNDiscrete);
        *pucRegBuffer = *pucRegBuffer >>(8 - usNDiscrete);
    } else {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

//eMBErrorCode
//eMBReportSlaveIdCB( UCHAR * pucRegBuffer, UCHAR ucNBytes ){
//    eMBErrorCode    eStatus = MB_ENOERR;
//    UCHAR *         pusRegRepSlaveIdBuf;
//
//    return eStatus;
//}





/**/
uint8_t xMbGetCoil( uint16_t usBitOffset ) {
    return xMBUtilGetBits( ucCoilBuf, usBitOffset, 1 );
}


/**/
void xMbSetCoil( uint16_t usBitOffset, uint8_t ucValue ) {
    xMBUtilSetBits( ucCoilBuf, usBitOffset, 1, ucValue );
}


/**/
uint8_t xMbGetDInput( uint16_t usBitOffset ) {
    return xMBUtilGetBits( ucDiscInputBuf, usBitOffset, 1 );
}


/**/
void xMbSetDInput( uint16_t usBitOffset, uint8_t ucValue ) {
    xMBUtilSetBits( ucDiscInputBuf, usBitOffset, 1, ucValue );
}


/**/
uint8_t xMbGetNCoils( uint16_t usBitOffset, uint8_t ucNBits ) {
    return xMBUtilGetBits( ucCoilBuf, usBitOffset, ucNBits );
}


/**/
void xMbSetNCoils( uint16_t usBitOffset, uint8_t ucNBits, uint8_t ucValue ) {
    xMBUtilSetBits( ucCoilBuf, usBitOffset, ucNBits, ucValue );
}

/**/
uint8_t xMbGetNDInputs( uint16_t usBitOffset, uint8_t ucNBits ) {
    return xMBUtilGetBits( ucDiscInputBuf, usBitOffset, ucNBits );
}


/**/
void xMbSetNDInputs( uint16_t usBitOffset, uint8_t ucNBits, uint8_t ucValue ) {
    xMBUtilSetBits( ucDiscInputBuf, usBitOffset, ucNBits, ucValue );
}

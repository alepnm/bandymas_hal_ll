

#include "hardware.h"


#define I2C_WRITE 0
#define I2C_READ  1

#define I2C_ADDR_NACK 1
#define I2C_OK 0




/*  */
void SPI_Receive8(uint8_t* rxdata, uint16_t len) {

    if( rxdata == NULL || len == 0 ) {
        ErrorCode = RES_BAD_PARAMS;
        Error_Handler();
    }

    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

    while(len > 0) {

        while( !LL_SPI_IsActiveFlag_TXE(SPI1) );
        LL_SPI_TransmitData8(SPI1, 0x00 );

        while( !LL_SPI_IsActiveFlag_RXNE(SPI1) );
        *rxdata = LL_SPI_ReceiveData8(SPI1);

        rxdata++;
        len--;
    }
}


/*  */
void SPI_Transmit8(uint8_t* txdata, uint16_t len){

    if(txdata == NULL || len == 0){
        ErrorCode = RES_BAD_PARAMS;
        Error_Handler();
    }

    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

    while(len > 0) {

        while( !LL_SPI_IsActiveFlag_TXE(SPI1) );
        LL_SPI_TransmitData8(SPI1, *txdata);

        while( !LL_SPI_IsActiveFlag_RXNE(SPI1) );
        (void)LL_SPI_ReceiveData8(SPI1);

        txdata++;
        len--;
    }
}


/*  */
void SPI_TransmitReceive8(uint8_t* txdata, uint8_t* rxdata, uint16_t len) {

    if( txdata == NULL || rxdata == NULL || len == 0 ) {
        ErrorCode = RES_BAD_PARAMS;
        Error_Handler();
    }

    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);

    while(len > 0) {

        while( !LL_SPI_IsActiveFlag_TXE(SPI1) );
        LL_SPI_TransmitData8(SPI1, *txdata );

        while( !LL_SPI_IsActiveFlag_RXNE(SPI1) );
        *rxdata = LL_SPI_ReceiveData8(SPI1);

        txdata++;
        rxdata++;
        len--;
    }
}



/*  */
uint16_t ADC_StartConversion(uint32_t channel, uint32_t resolution) {

    uint16_t result;

    ADC1->CHSELR = channel;
    LL_ADC_SetResolution(ADC1, resolution);

    do{
        LL_ADC_Enable(ADC1);
    }while ( !LL_ADC_IsActiveFlag_ADRDY(ADC1) );

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

    do{
        LL_ADC_Disable(ADC1);
    }while( LL_ADC_IsEnabled(ADC1) );

    return result;
}



/*  */
void IIC_Start(void){

    do{
        LL_I2C_Enable(I2C1);
    }while( !LL_I2C_IsEnabled(I2C1) );
}


/*  */
void IIC_Process( uint16_t mem_addr, void *data, uint16_t len, uint8_t proc ){



//    LL_I2C_HandleTransfer(I2C1, 0xA0, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);




//    LL_I2C_GenerateStartCondition(I2C1);
//
//
//    LL_I2C_SetSlaveAddr(I2C1, 0xA0);
//    LL_I2C_SetTransferRequest(I2C1, LL_I2C_REQUEST_READ);
//
//    LL_I2C_SetTransferSize(I2C1, len);
//
//
//
//
//    LL_I2C_GenerateStopCondition(I2C1);

}



uint8_t IIC_Read(uint8_t Addr, uint8_t *data, uint8_t reg, uint8_t len){

    LL_I2C_HandleTransfer(I2C1, (Addr<<1), LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    /* Loop until end of transfer received */
    while( LL_I2C_IsActiveFlag_STOP(I2C1) == RESET ){

        /* Indicate the status of Transmit data register empty flag */
        if( LL_I2C_IsActiveFlag_TXE(I2C1) != RESET ){

            /* Write data in Transmit Data register. */
            LL_I2C_TransmitData8(I2C1, reg);
        }
    }

    LL_I2C_ClearFlag_STOP(I2C1);


    LL_I2C_HandleTransfer(I2C1, (Addr<<1), LL_I2C_ADDRSLAVE_7BIT, len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    /* Loop until end of transfer received */
    while( LL_I2C_IsActiveFlag_STOP(I2C1) == RESET ){

        /*Check RXNE flag value in ISR register*/
        if( LL_I2C_IsActiveFlag_RXNE(I2C1) != RESET ){

            /*Read character in Receive Data register.*/
            *data = LL_I2C_ReceiveData8(I2C1);
        }
    }

    LL_I2C_ClearFlag_STOP(I2C1);

    /* all is good, return 0 */
    return 0;
}


uint8_t I2C_read_brekeke(I2C_TypeDef *I2Cx, unsigned char Address, uint8_t reg, int nBytes, unsigned char *data)
{
    LL_I2C_SetSlaveAddr(I2Cx, (Address<<1)); /* Prepare Address to send */
    LL_I2C_SetMasterAddressingMode(I2Cx, LL_I2C_ADDRSLAVE_7BIT);
    LL_I2C_SetTransferRequest(I2Cx, LL_I2C_REQUEST_WRITE); /* Reguest write */
    LL_I2C_DisableAutoEndMode(I2Cx); /* Disable automatic STOP condition generation */
    LL_I2C_SetTransferSize(I2Cx, 1); /* Set transfer size register */

    while(LL_I2C_IsActiveFlag_BUSY(I2Cx)); /* check I2C bussy */

    LL_I2C_GenerateStartCondition(I2Cx); /* generate I2C Start address and send address*/

    while(!LL_I2C_IsActiveFlag_TC(I2Cx)) /* check STOP bit */
    {
        if(LL_I2C_IsActiveFlag_TXE(I2Cx))
        {
            LL_I2C_TransmitData8(I2Cx, reg); /* send data out */
        }
    }

    LL_I2C_SetSlaveAddr(I2Cx, (Address<<1)); /* Prepare Address to send */
    LL_I2C_SetMasterAddressingMode(I2Cx, LL_I2C_ADDRSLAVE_7BIT);
    LL_I2C_SetTransferRequest(I2Cx, LL_I2C_REQUEST_READ); /* Reguest write */
    LL_I2C_DisableAutoEndMode(I2Cx); /* Enable autoend mode*/
    LL_I2C_SetTransferSize(I2Cx, nBytes); /* Set transfer size register */

    //while(LL_I2C_IsActiveFlag_BUSY(I2Cx)); /* check I2C bussy */
    LL_I2C_GenerateStartCondition(I2Cx); /* generate I2C Start address and send address*/
    while(!LL_I2C_IsActiveFlag_TC(I2Cx)) /* check STOP bit */
    {
        if(LL_I2C_IsActiveFlag_RXNE(I2Cx))
        {
            *data++ = LL_I2C_ReceiveData8(I2Cx); /* send data out */
        }
    }

    LL_I2C_GenerateStopCondition(I2Cx);

    return 0;
}




uint8_t I2C_HW_Write(uint8_t Addr, uint16_t Reg, uint8_t Value)
{
    LL_I2C_HandleTransfer(I2C1, Addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    LL_I2C_TransmitData8(I2C1, 0x00);
    while(!LL_I2C_IsActiveFlag_TXE(I2C1))
    {
        if(LL_SYSTICK_IsActiveCounterFlag() && LL_I2C_IsActiveFlag_NACK(I2C1))
        {
            return I2C_ADDR_NACK;
        }
    }
    LL_I2C_ClearFlag_STOP(I2C1);
    LL_I2C_HandleTransfer(I2C1, Addr, LL_I2C_ADDRSLAVE_7BIT, 3, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while(LL_I2C_IsActiveFlag_ADDR(I2C1))
    {
    }
    LL_I2C_TransmitData8(I2C1, ((Reg & 0xFF00) >> 8));
    while(!LL_I2C_IsActiveFlag_TXE(I2C1))
    {
    }
    LL_I2C_TransmitData8(I2C1, (Reg & 0x00FF));
    while(!LL_I2C_IsActiveFlag_TXE(I2C1))
    {
    }
    LL_I2C_TransmitData8(I2C1, Value);
    while(!LL_I2C_IsActiveFlag_TXE(I2C1))
    {
    }
    LL_I2C_ClearFlag_STOP(I2C1);

    return I2C_OK;
}


uint8_t I2C_HW_Read(uint8_t Addr, uint16_t Reg, uint8_t *Value)
{
    LL_I2C_HandleTransfer(I2C1, Addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    LL_I2C_TransmitData8(I2C1, 0x00);
    while(!LL_I2C_IsActiveFlag_TXE(I2C1))
    {
        if(LL_SYSTICK_IsActiveCounterFlag() && LL_I2C_IsActiveFlag_NACK(I2C1))
        {
            return I2C_ADDR_NACK;
        }
    }
    LL_I2C_ClearFlag_STOP(I2C1);
    LL_I2C_HandleTransfer(I2C1, Addr, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
    while(LL_I2C_IsActiveFlag_ADDR(I2C1))
    {
    }
    LL_I2C_TransmitData8(I2C1, ((Reg & 0xFF00) >> 8));
    while(!LL_I2C_IsActiveFlag_TXE(I2C1))
    {
    }
    LL_I2C_TransmitData8(I2C1, (Reg & 0x00FF));
    while(!LL_I2C_IsActiveFlag_TXE(I2C1))
    {
    }
    LL_I2C_HandleTransfer(I2C1, Addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_RESTART_7BIT_READ);
    while(!LL_I2C_IsActiveFlag_RXNE(I2C1))
    {
    }
    *Value = LL_I2C_ReceiveData8(I2C1);
    LL_I2C_ClearFlag_STOP(I2C1);

    return I2C_OK;
}

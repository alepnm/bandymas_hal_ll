

#include "hardware.h"

#define I2C_WRITE 0
#define I2C_READ  1

#define I2C_ADDR_NACK 1
#define I2C_OK 0


extern volatile uint32_t timestamp;

/*  */
void Delay_ms(uint32_t delay){

    delay += timestamp;

    while(delay > timestamp);
}

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
uint8_t IIC_Write(uint8_t iic_addr, uint16_t reg, uint16_t len, uint8_t *buf)
{
    LL_I2C_HandleTransfer(I2C1, iic_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    LL_I2C_TransmitData8(I2C1, 0x00);

    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET ){
        if(LL_SYSTICK_IsActiveCounterFlag() && LL_I2C_IsActiveFlag_NACK(I2C1))
        {
            return I2C_ADDR_NACK;
        }
    }

    LL_I2C_ClearFlag_STOP(I2C1);

    LL_I2C_HandleTransfer(I2C1, iic_addr, LL_I2C_ADDRSLAVE_7BIT, len+2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    while( LL_I2C_IsActiveFlag_ADDR(I2C1) != RESET );

    LL_I2C_TransmitData8(I2C1, ((reg & 0xFF00) >> 8));
    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );

    LL_I2C_TransmitData8(I2C1, (reg & 0x00FF));
    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );

    do{
        LL_I2C_TransmitData8(I2C1, *buf);
        while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );
        buf++;
    }while(--len > 0);

    LL_I2C_ClearFlag_STOP(I2C1);

    return I2C_OK;
}


/*  */
uint8_t IIC_Read(uint8_t iic_addr, uint16_t reg, uint16_t len, uint8_t *buf)
{
    LL_I2C_HandleTransfer(I2C1, iic_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    LL_I2C_TransmitData8(I2C1, 0x00);

    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET ){
        if(LL_SYSTICK_IsActiveCounterFlag() && LL_I2C_IsActiveFlag_NACK(I2C1))
        {
            return I2C_ADDR_NACK;
        }
    }

    LL_I2C_ClearFlag_STOP(I2C1);

    LL_I2C_HandleTransfer(I2C1, iic_addr, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    while( LL_I2C_IsActiveFlag_ADDR(I2C1) != RESET );

    LL_I2C_TransmitData8(I2C1, ((reg & 0xFF00) >> 8));
    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );

    LL_I2C_TransmitData8(I2C1, (reg & 0x00FF));
    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );

    Delay_ms(5);    //<-- butinas uzdelsimas

    LL_I2C_HandleTransfer(I2C1, iic_addr, LL_I2C_ADDRSLAVE_7BIT, len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    do{
        while( LL_I2C_IsActiveFlag_RXNE(I2C1) == RESET );
        *buf = LL_I2C_ReceiveData8(I2C1);
        buf++;
    }while(--len > 0);

    LL_I2C_ClearFlag_STOP(I2C1);

    return I2C_OK;
}


/*  */
uint8_t IIC_ReadByte(uint16_t reg){

    uint8_t data;

    (void)IIC_Read(0xA0, reg, 1, &data);

    return data;
}


/*  */
uint16_t IIC_ReadWord(uint16_t reg){

    uint8_t data[2];

    (void)IIC_Read(0xA0, reg, 2, data);

    return (data[1]<<8 | data[0] );
}


/*  */
uint32_t IIC_ReadDWord(uint32_t reg){

    uint8_t data[4];

    (void)IIC_Read(0xA0, reg, 4, data);

    return ( data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24 );
}

/*  */
void IIC_WriteByte(uint16_t reg, uint8_t val){
    (void)IIC_Write(0xA0, reg, 1, &val);
}

/*  */
void IIC_WriteWord(uint16_t reg, uint16_t val){

    uint8_t data[2]={ val & 0xFF, val >> 8 };

    (void)IIC_Write(0xA0, reg, 2, data);
}

/*  */
void IIC_WriteDWord(uint16_t reg, uint32_t val){

    uint8_t data[4]={ val & 0xFF, val >> 8, val >> 16, val >> 24 };

    (void)IIC_Write(0xA0, reg, 4, data);
}

#include "main.h"
#include "iic.h"

#define     I2C_WRITE 0
#define     I2C_READ  1

#define     I2C_ADDR_NACK   1
#define     I2C_OK          0


/*  */
uint8_t IIC_Check(uint8_t iic_addr) {

    LL_I2C_HandleTransfer(I2C1, iic_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

    LL_I2C_TransmitData8(I2C1, 0x00);

    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET ) {
        if(LL_SYSTICK_IsActiveCounterFlag() && LL_I2C_IsActiveFlag_NACK(I2C1)) {
            return I2C_ADDR_NACK;
        }
    }

    LL_I2C_ClearFlag_STOP(I2C1);

    return I2C_OK;
}

/* galima rasyti iki 254 baitu! */
uint8_t IIC_Write(uint8_t iic_addr, uint16_t reg, uint8_t *buf, uint8_t len) {

    uint8_t wr_cnt = 0, tmp = 0;

    LL_I2C_ClearFlag_STOP(I2C1);

    do{

        wr_cnt = (len > 254) ? 254 : len;

        tmp = wr_cnt;

        LL_I2C_HandleTransfer(I2C1, iic_addr, LL_I2C_ADDRSLAVE_7BIT, wr_cnt+1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
        while( LL_I2C_IsActiveFlag_ADDR(I2C1) != RESET );

        LL_I2C_TransmitData8(I2C1, reg);
        while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );

        do{
            LL_I2C_TransmitData8(I2C1, *buf);
            while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );
            buf++;
        }while(--wr_cnt > 0);

        len -= tmp;
        reg += tmp;

        LL_I2C_ClearFlag_STOP(I2C1);

        Delay_ms(5);

    }while(len > 0);

    return I2C_OK;
}


/*  */
uint8_t IIC_Read(uint8_t iic_addr, uint16_t reg, uint8_t *buf, uint8_t len) {

    LL_I2C_ClearFlag_STOP(I2C1);

    LL_I2C_HandleTransfer(I2C1, iic_addr, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while( LL_I2C_IsActiveFlag_ADDR(I2C1) != RESET );

    LL_I2C_TransmitData8(I2C1, reg);
    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );

    LL_I2C_ClearFlag_STOP(I2C1);

    LL_I2C_HandleTransfer(I2C1, iic_addr, LL_I2C_ADDRSLAVE_7BIT, len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    do {
        while( LL_I2C_IsActiveFlag_RXNE(I2C1) == RESET );
        *buf = LL_I2C_ReceiveData8(I2C1);
        buf++;
    } while(--len > 0);

    LL_I2C_ClearFlag_STOP(I2C1);

    return I2C_OK;
}


/*  */
uint8_t IIC_ReadByte(uint16_t reg) {

    LL_I2C_HandleTransfer(I2C1, 0xA0, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while( LL_I2C_IsActiveFlag_ADDR(I2C1) != RESET );

    LL_I2C_TransmitData8(I2C1, reg);
    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );

    LL_I2C_ClearFlag_STOP(I2C1);

    LL_I2C_HandleTransfer(I2C1, 0xA0, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

    while( LL_I2C_IsActiveFlag_RXNE(I2C1) == RESET );
    uint8_t data = LL_I2C_ReceiveData8(I2C1);

    LL_I2C_ClearFlag_STOP(I2C1);

    return data;
}


/*  */
void IIC_WriteByte(uint16_t reg, uint8_t data) {

    LL_I2C_HandleTransfer(I2C1, 0xA0, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while( LL_I2C_IsActiveFlag_ADDR(I2C1) != RESET );

    LL_I2C_TransmitData8(I2C1, reg);
    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );

    LL_I2C_TransmitData8(I2C1, data);
    while( LL_I2C_IsActiveFlag_TXE(I2C1) == RESET );

    LL_I2C_ClearFlag_STOP(I2C1);

    Delay_ms(5);
}

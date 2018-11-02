#include "usart.h"
#include "mbport.h"

#define USART_PAR_NONE  MB_PAR_NONE
#define USART_PAR_ODD   MB_PAR_ODD
#define USART_PAR_EVEN  MB_PAR_EVEN

USART_TypeDef* ports[3u] = {USART1, NULL, NULL};

enum { BR2400 = 0, BR4800, BR9600, BR19200, BR38400, BR57600 };
const uint32_t baudrates[6u] = { 2400u, 4800u, 9600u, 19200u, 38400u, 57600u };


uint32_t    Usart_ConfigRegister;
/*
     0-15 - Baudrate
    16-17 - Parity
    18-19 - StopBits
    20-22 - DataBits
    23-25 - baudreito indeksas is bodreitu masyvo
    26-27 - naudojamo USART porto numeris
*/

#define USART_BAUDRATE_MSK          0b00000000000000001111111111111111
#define USART_PARITY_MSK            0b00000000000000110000000000000000
#define USART_STOPBITS_MSK          0b00000000000011000000000000000000
#define USART_DATABITS_MSK          0b00000000011100000000000000000000
#define USART_BAUDRATE_IDX_MSK      0b00000011100000000000000000000000
#define USART_PORT_NUMBER_MSK       0b00001100000000000000000000000000

#define USART_GET_BAUDRATE          (Usart_ConfigRegister & USART_BAUDRATE_MSK)
#define USART_SET_BAUDRATE(x)       Usart_ConfigRegister ^= (Usart_ConfigRegister & USART_BAUDRATE_MSK); Usart_ConfigRegister |= x
#define USART_GET_PARITY            (Usart_ConfigRegister & USART_PARITY_MSK)>>16
#define USART_SET_PARITY(x)         Usart_ConfigRegister ^= (Usart_ConfigRegister & USART_PARITY_MSK); Usart_ConfigRegister |= (x<<16)
#define USART_GET_STOPBITS          (Usart_ConfigRegister & USART_STOPBITS_MSK)>>18
#define USART_SET_STOPBITS(x)       Usart_ConfigRegister ^= (Usart_ConfigRegister & USART_STOPBITS_MSK); Usart_ConfigRegister |= (x<<18)
#define USART_GET_DATABITS          (Usart_ConfigRegister & USART_DATABITS_MSK)>>20
#define USART_SET_DATABITS(x)       Usart_ConfigRegister ^= (Usart_ConfigRegister & USART_DATABITS_MSK); Usart_ConfigRegister |= (x<<20)
#define USART_GET_BAUDRATE_IDX      (Usart_ConfigRegister & USART_BAUDRATE_IDX_MSK)>>23
#define USART_SET_BAUDRATE_IDX(x)   Usart_ConfigRegister ^= (Usart_ConfigRegister & USART_BAUDRATE_IDX_MSK); Usart_ConfigRegister |= (x<<23)
#define USART_GET_PORT_NUMBER       (Usart_ConfigRegister & USART_PORT_NUMBER_MSK)>>26
#define USART_SET_PORT_NUMBER(x)    Usart_ConfigRegister ^= (Usart_ConfigRegister & USART_PORT_NUMBER_MSK); Usart_ConfigRegister |= (x<<26)

extern LL_USART_InitTypeDef USART_InitStruct;



/*  */
void USART_Config(uint8_t ucPORT, uint32_t ulBaudRate, uint32_t ulDataBits,  uint32_t ulParity ) {

    UNUSED(ulDataBits);

    do{
         LL_USART_Disable(ports[ucPORT]);
    }while( LL_USART_IsEnabled(ports[ucPORT]) );

    /* cia reiketu patikrinti baudreito reiksme - ar ji standartine? */
    USART_InitStruct.BaudRate = ulBaudRate;

    switch(ulParity) {
    case USART_PAR_ODD:
        USART_InitStruct.Parity = LL_USART_PARITY_ODD;
    case USART_PAR_EVEN:
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

/*  */
void USART_SendString(const char* str){

    uint8_t i = 0;

    while(*str > 0x00){

        LL_USART_TransmitData8(USART1, *str+i);
        i++;
    }
}
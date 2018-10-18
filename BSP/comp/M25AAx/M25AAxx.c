/*

*/
#include "M25AAxx.h"
#include "hardware.h"
#include "stepper.h"


/* **************************** DEFINES, MACRO ***************************** */
#define MEMORY_SIZE                         M25AAxx_EE_SIZE
#define PAGE_SIZE                           16
#define M25AAxx_EE_SIZE                     192


#ifdef M25AA02E48
    #define M25AAxx_UID_NODE_ADDRESS        0xFA
#else
    #define M25AAxx_UID_NODE_ADDRESS        0xF8
#endif


#define WRITE_STATUS_INSTRUCTION            0x01    // 0b00000001
#define WRITE_INSTRUCTION                   0x02    // 0b00000010
#define READ_INSTRUCTION                    0x03    // 0b00000011
#define WRITE_DISABLE_INSTRUCTION           0x04    // 0b00000100
#define READ_STATUS_INSTRUCTION             0x05    // 0b00000101
#define WRITE_ENABLE_INSTRUCTION            0x06    // 0b00000110


/* Status register bits */
#define M25AAxx_STATUS_WIP                  0x01    // bit WIP - Write In Process (RO)
#define M25AAxx_STATUS_WEL                  (0x01)<<1    // bit WEL - Write Enable Latch (RO)

#define M25AAxx_STATUS_BP_MASK              0x0C        // mask for BP bits
#define M25AAxx_STATUS_BP0                  (0x01)<<2    // bit BP0 - Block0 Protection (RW)
#define M25AAxx_STATUS_BP1                  (0x01)<<3    // bit BP1 - Block1 Protection (RW)


#define CS_LOW()                            M25AA_CS_LOW()
#define CS_HIGH()                           M25AA_CS_HIGH()


M25AAxx_TypeDef M25AAxx;

/* ****************************    GLOBALS     ***************************** */
/* ****************************    PRIVATES    ***************************** */
static eRESULT_TypeDef result = RES_OK;
/* *************************** LOCAL FUNCTIONS ***************************** */
static uint8_t  Read( uint8_t addr, uint8_t *buffer, int len );
static uint8_t  Write( uint8_t addr, uint8_t* buffer, uint8_t len );
static uint8_t  WriteEnable( void );
static uint8_t  WriteDisable( void );
static bool     GetWriteFlag( );

static uint8_t  PullStatusRegister( uint8_t* status );
static uint8_t  PushStatusRegister( uint8_t* status );

static uint8_t  spi_send(uint8_t* pData, uint8_t len);
static uint8_t  spi_receive(uint8_t* pData, uint8_t len);

/* **************************** IMPLEMENTATION ***************************** */


/*  */
uint8_t M25AAxx_Init(void){

    M25AAxx.UidBufferSize = M25AAxx_UID_BUFFER_SIZE;

    WriteDisable();
    M25AAxx_ReadUID(M25AAxx.UidBuffer);

    return 0;
}


/* Skaitymas UID */
uint8_t M25AAxx_ReadUID( unsigned char* buffer) {

    if( buffer == NULL ) return 1;

    uint8_t data_tx = (uint8_t)READ_INSTRUCTION;

    CS_LOW();

    if( (result = spi_send(&data_tx, 1) ) != 0 ) {
        CS_HIGH();
        return 1;
    };

    data_tx = M25AAxx_UID_NODE_ADDRESS;

    if( (result = spi_send(&data_tx, 1) ) != 0 ) {
        CS_HIGH();
        return 1;
    };

    result = spi_receive(buffer, M25AAxx_UID_BUFFER_SIZE);

    CS_HIGH();

    return result;
}

/* Skaitymas Byte */
uint8_t M25AAxx_ReadByte(uint8_t addr) {

    unsigned char reg;

    (void)Read( addr, (uint8_t*)&reg, 1 );

    return reg;
}

/* Skaitymas Word */
uint16_t  M25AAxx_ReadWord(uint8_t addr) {

    union {
        unsigned char data[2];
        unsigned int intval;
    } reg;

    (void)Read( addr, reg.data, 2 );

    return ( reg.intval );
}

/* Skaitymas Double Word */
uint32_t M25AAxx_ReadDWord(uint8_t addr) {

    union {
        unsigned char data[4];
        unsigned long intval;
    } reg;

    (void)Read( addr, reg.data, 4 );

    return ( reg.intval );
}

/* Skaitymas i buferi N baitu */
uint8_t M25AAxx_Read( uint8_t addr, uint8_t* data, uint8_t len ) {
    return ( Read( addr, data, len ) != 0 ) ? 1 : 0;
}

/* Irasymas Byte */
uint8_t M25AAxx_WriteByte( uint8_t addr, uint8_t value ) {
    return ( Write( addr, &value, 1 ) != 0 ) ? 1 : 0;
}

/* Irasymas Word */
uint8_t M25AAxx_WriteWord( uint8_t addr, uint16_t value ) {

    union {
        uint8_t data[2];
        uint16_t intval;
    } reg;

    reg.intval = value;

    return ( Write( addr, reg.data, 2 ) != 0 ) ? 1 : 0;
}

/* Irasymas Double Word */
uint8_t M25AAxx_WriteDWord( uint8_t addr, uint32_t value ) {

    union {
        uint8_t data[4];
        uint32_t intval;
    } reg;

    reg.intval = value;

    return ( Write( addr, reg.data, 4 ) != 0 ) ? 1 : 0;
}

/* Irasymas is buferio N baitu */
uint8_t M25AAxx_Write(uint8_t addr, uint8_t* data, uint8_t len) {

    return ( Write( addr, data, len ) != 0 ) ? 1 : 0;
}


/* Isvalom EEPROM */
uint8_t M25AAxx_Clear(void) {

    uint8_t* data = (uint8_t*)malloc(sizeof(uint8_t)*(PAGE_SIZE));
    uint16_t i = 0;
    uint8_t addr = 0;

    do {

        *(data+i) = 0xFF;

    } while(++i < PAGE_SIZE);

    while(addr < MEMORY_SIZE) {

        if( (result = Write( addr, data, PAGE_SIZE )) != 0 ) break;
        addr += PAGE_SIZE;
    }

    free(data);

    return result;
}

/*  */
uint8_t M25AAxx_BlockProtect(uint8_t block){

    if(block == 0||block > 3) return 1;

    uint8_t status = 0;

    PullStatusRegister(&status);

    status |= (block<<2);

    PushStatusRegister(&status);

    return 0;
}

/*  */
uint8_t M25AAxx_BlockUnProtect(uint8_t block){

    if(block == 0||block > 3) return 1;

    uint8_t status = 0;

    PullStatusRegister(&status);

    status &= ~(block<<2);

    PushStatusRegister(&status);

    return 0;
}


/* Skaitom Status Registra */
static uint8_t PullStatusRegister( uint8_t* status ) {

    uint8_t data_tx = (uint8_t)READ_STATUS_INSTRUCTION;

    CS_LOW();

    if( spi_send(&data_tx, 1) != 0 || spi_receive(status, 1) != 0 ) {
        result = 1;
    }

    CS_HIGH();

    return result;
}


/* Rasom Status registra */
static uint8_t PushStatusRegister( uint8_t* status ) {

    uint8_t data_tx = (uint8_t)WRITE_STATUS_INSTRUCTION;

    (void)WriteEnable();

    CS_LOW();

    if( spi_send(&data_tx, 1) != 0 || spi_send(status, 1) != 0 ) {
        result = 1;
    }

    CS_HIGH();

    return result;
}

/*  */
static bool GetWriteFlag( ) {

    uint8_t status;

    (void)PullStatusRegister(&status);

    return (bool)( status & M25AAxx_STATUS_WIP );
}


/*  */
static uint8_t Read( uint8_t addr, uint8_t* buffer, int len ) {

    if( addr > MEMORY_SIZE-1 || buffer == NULL || !len || len > MEMORY_SIZE ) return 1;

    uint8_t cmd = (uint8_t)READ_INSTRUCTION;

    CS_LOW();

    if( spi_send(&cmd, 1) != 0 || spi_send(&addr, 1) != 0 || spi_receive(buffer, len) != 0 ) {
        result = 1;
    }

    CS_HIGH();

    return result;
}


/* rasymas is buferio N baitu */
static uint8_t Write( uint8_t addr, uint8_t* buffer, uint8_t len ) {

    uint8_t cmd = (uint8_t)WRITE_INSTRUCTION;

    if( addr > MEMORY_SIZE-1 || buffer == NULL || !len || len > MEMORY_SIZE ) return 1;

    uint8_t wr_size = 0;
    uint8_t offset_in_page = addr & (PAGE_SIZE-1);

    if( offset_in_page+len > PAGE_SIZE ) {

        while ( len > 0 ) {

            wr_size = PAGE_SIZE - offset_in_page;

            if(len < wr_size) wr_size = len;

            (void)WriteEnable();

            CS_LOW();

            if( spi_send(&cmd, 1) != 0 || spi_send(&addr, 1) != 0 || spi_send(buffer, wr_size) != 0 ) {

                CS_HIGH();

                HW_Delay(7);

                return 1;
            }

            CS_HIGH();

            while( GetWriteFlag() != RESET );

            offset_in_page = 0;
            buffer += wr_size;
            addr += wr_size;
            len -= wr_size;

            HW_Delay(7);
        }

        return result;
    }

    (void)WriteEnable();

    CS_LOW();

    if( spi_send(&cmd, 1) != 0 || spi_send(&addr, 1) != 0 || spi_send(buffer, len) != 0 ) {
        result = 1;
    }

    CS_HIGH();

    while( GetWriteFlag() != RESET );

    HW_Delay(7);

    return result;
}


/* vykdom pries kiek viena irasyma */
static uint8_t WriteEnable() {

    uint8_t cmd = (uint8_t)WRITE_ENABLE_INSTRUCTION;

    CS_LOW();

    result = spi_send(&cmd, 1);

    CS_HIGH();

    return result;
}


/* vykdyti nereikia - po sekmingos write komandos atsistato automatiskai*/
static uint8_t WriteDisable() {

    uint8_t cmd = (uint8_t)WRITE_DISABLE_INSTRUCTION;

    CS_LOW();

    result = spi_send(&cmd, 1);

    CS_HIGH();

    return result;
}


inline static uint8_t spi_send(uint8_t* pData, uint8_t len) {
    SPI_Transmit8(pData, (uint16_t)len);
    return 0;
}

inline static uint8_t spi_receive(uint8_t* pData, uint8_t len) {
    SPI_Receive8(pData, (uint16_t)len);
    return 0;
}

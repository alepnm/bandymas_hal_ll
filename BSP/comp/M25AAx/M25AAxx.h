/*

*/

#ifndef MICROCHIP_25AA02_H
#define MICROCHIP_25AA02_H

#include <stdint.h>

#define M25AA02E48

#ifdef M25AA02E48
    #define M25AAxx_UID_BUFFER_SIZE             0x06
#else
    #define M25AAxx_UID_BUFFER_SIZE             0x08
#endif

#define EE_INIT_BYTE                    0x55

typedef struct{
    uint8_t UidBufferSize;
    uint8_t UidBuffer[M25AAxx_UID_BUFFER_SIZE];
}M25AAxx_TypeDef;

extern M25AAxx_TypeDef M25AAxx;


uint8_t     M25AAxx_Init(void);
uint8_t     M25AAxx_ReadUID( unsigned char* buffer);
uint8_t     M25AAxx_ReadByte(uint8_t addr);
uint16_t    M25AAxx_ReadWord(uint8_t addr);
uint32_t    M25AAxx_ReadDWord(uint8_t addr);
uint8_t     M25AAxx_Read( uint8_t addr, uint8_t* data, uint8_t len );
uint8_t     M25AAxx_WriteByte( uint8_t addr, uint8_t value );
uint8_t     M25AAxx_WriteWord( uint8_t addr, uint16_t value );
uint8_t     M25AAxx_WriteDWord( uint8_t addr, uint32_t value );
uint8_t     M25AAxx_Write(uint8_t addr, uint8_t* data, uint8_t len);
uint8_t     M25AAxx_BlockProtect(uint8_t block);
uint8_t     M25AAxx_BlockUnProtect(uint8_t block);
uint8_t     M25AAxx_Clear(void);



#endif // MICROCHIP_25AA02_H

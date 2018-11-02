#include <stdlib.h>
#include <string.h>

#include "iic_eeprom.h"
#include "iic.h"

enum {IIC_READ = 0, IIC_WRITE} eIIC_PROCESS;  // kaip komunikuojam su iic

/*  Prototipai  */
static uint8_t EEP24XX_Process( uint16_t mem_addr, void *txdata, uint16_t len, uint8_t proc );


/* rasymas i EEPROM pool rezime */
uint8_t EEP24XX_Write( uint16_t mem_addr, void *data, size_t size_of_data ) {

    if( mem_addr > I2C_MEMORY_SIZE-1 || data == NULL || size_of_data == 0 ) return RES_BAD_PARAMS;    //error

    return EEP24XX_Process( mem_addr, data, size_of_data, IIC_WRITE );
}

/* skaitymas is EEPROM pool rezime */
uint8_t EEP24XX_Read( uint16_t mem_addr, void *rxdata, size_t size_of_data ) {

    if( mem_addr > I2C_MEMORY_SIZE-1 || rxdata == NULL || size_of_data == 0 ) return RES_BAD_PARAMS;    //error

    return EEP24XX_Process( mem_addr, rxdata, size_of_data, IIC_READ );
}

/*   */
uint8_t EEP24XX_ReadByte(uint16_t mem_addr){
    return IIC_ReadByte(mem_addr);
}

/*  */
uint16_t EEP24XX_ReadWord(uint16_t mem_addr){

    uint8_t data[2];

    EEP24XX_Process( mem_addr, data, 2, IIC_READ );

    return ( data[1]<<8 | data[0] );
}

/*  */
uint32_t EEP24XX_ReadDWord(uint16_t mem_addr){

    uint8_t data[4];

    EEP24XX_Process( mem_addr, data, 4, IIC_READ );

    return ( data[3]<<24 | data[2]<<16 | data[1]<<8 | data[0] );
}

/*   */
void EEP24XX_WriteByte(uint16_t mem_addr, uint8_t val){
    IIC_WriteByte(mem_addr, val);
}

/*  */
void EEP24XX_WriteWord(uint16_t mem_addr, uint16_t val){

    uint8_t data[2] = { (uint8_t)(val&0x00FF), (uint8_t)((val>>8)&0x00FF) };

    EEP24XX_Process( mem_addr, data, 2, IIC_WRITE );
}

/*  */
void EEP24XX_WriteDWord(uint16_t mem_addr, uint32_t val){

    uint8_t data[4] = { (uint8_t)(val&0x000000FF), (uint8_t)((val>>8)&0x000000FF), (uint8_t)((val>>16)&0x000000FF), (uint8_t)((val>>24)&0x000000FF) };

    EEP24XX_Process( mem_addr, data, 4, IIC_WRITE );
}

/*  */
uint8_t EEP24XX_Clear(void) {

    uint8_t result = RES_OK;
    uint8_t *data = (uint8_t*)malloc(sizeof(uint8_t)*(PAGE_SIZE));
    uint16_t i = 0;
    uint16_t addr = 0;

    do {
        *(data+i) = 0xFF;
    } while(++i < PAGE_SIZE);

    while(addr < I2C_MEMORY_SIZE) {
        if( (result = EEP24XX_Process( addr, data, PAGE_SIZE, IIC_WRITE )) != RES_OK) break;
        addr += PAGE_SIZE;
    }

    free(data);

    return result;
}

/* EEPROM processas */
static uint8_t EEP24XX_Process( uint16_t mem_addr, void *data, uint16_t len, uint8_t proc ) {

    uint8_t result = RES_OK;

    uint8_t wr_size = 0;

    /* apskaiciuojam pagal adresa eeprom bloka, peidza */
    uint8_t block = mem_addr>>BLOCK_CALC_SHIFT_VAL;                 // kuris blokas
    //uint8_t pages = (len>>PAGE_CALC_SHIFT_VAL)+1;                   // kiek peidzu
    uint8_t page = mem_addr>>PAGE_CALC_SHIFT_VAL;                   // kuris peidzas eeprome
    uint8_t page_in_block = page - (block<<PAGE_CALC_SHIFT_VAL);    // kuris peidzas bloke
    uint8_t offset_in_page = mem_addr & (PAGE_SIZE-1);              // baito offsetas peidze nuo peidzio pradzios

    uint8_t eeaddr = (I2C_EEP_BASE_ADDRESS + block)<<1;

    if( ( result = IIC_Check(eeaddr) ) != RES_OK ) return result;

    while(len > 0) {

        wr_size = PAGE_SIZE - offset_in_page;   // galimas duomenu kiekis iki peidzo pabaigos

        if(len < wr_size) wr_size = len;  // jai likusiu duomenu maziau, nei vietos iki paidzo pabaigos, rasom tik likusius duomenys

        // irasom duomenu dali
        switch(proc){
            case IIC_READ:
                IIC_Read(eeaddr, mem_addr, data, (uint16_t)wr_size);
            break;
            case IIC_WRITE:
                IIC_Write(eeaddr, mem_addr, data, (uint16_t)wr_size);
                //Delay_ms(5);
            break;
        }

        /* cia jau esame peidzo gale, reikia pradeti nuo naujo peidzo */
        len -= wr_size;        // mazinam likusiu duomenu kieki

        if(len == 0) break;    // iseinam, jai viska irasem

        page++;
        offset_in_page = 0;
        mem_addr += wr_size;            // didinam likusiu duomenu irasymo adresa
        data += wr_size;               // perskaiciuojam pounteri i duomenys buferyje

        if(page_in_block == PAGES_IN_BLOCK-1) { // paskutinis peidzas bloke
            /* perjungiam bloka */
            block++;
            eeaddr = (I2C_EEP_BASE_ADDRESS + block)<<1;
            page_in_block = 0;
        } else {
            page_in_block++;        // pakeliam paidzo numeri bloke
        }
    }

    /* pagalvoti kaip ir ka grazinam!!! */
    return RES_OK;
}


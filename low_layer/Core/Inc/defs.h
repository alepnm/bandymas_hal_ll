#ifndef DEFS_H_INCLUDED
#define DEFS_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>


typedef enum { RES_OK = 0, RES_ERROR, RES_BUSY, RES_TIMEOUT, RES_BAD_PARAMS } eRESULT_TypeDef;  // atitinka HAL_StatusTypeDef

/* darbo rezimai */
typedef enum { MODE_MANUAL = 0, MODE_MODBUS, MODE_STEPCLOCK, MODE_TESTMODE } eMode_TypeDef;

/*  SYSTEM DEFAULTS */
#define     MODBUS_ENABLE

#define     MB_PORT_DEF                 ( 0u )

#define     MBADDR_DEF                  ( 10u )     //0x0A
#define     MBPARITY_DEF                MB_PAR_NONE
#define     MBBAURATE_DEF               ( 3u )      // bodreito indeksas lenteleje ( 3->19200 )
#define     MBSTOPBITS_DEF              ( 1u )
#define     MBWORDLENGHT_DEF            ( 8u )
#define     SOUND_LEVEL_DEF             SND_OFF
#define     WDT_FUNC_DEF                DISABLE

#define     RPM_MIN_DEF                 ( 10u )     // minimalus sukimosi greitis (RPM)
#define     RPM_MAX_DEF                 ( 200u )    // maksimalus sukimosi greitis (RPM)
#define     MICROSTEPS_DEF              ( 16u )     // mikrostepu
#define     MIN_TRES_OCD_MA_DEF         ( 500u )    // minimali sroves reiksme, mA
#define     MAX_TRES_OCD_MA_DEF         ( 4000u )   // maksimali sroves reiksme, mA
#define     MAX_KVAL_VALUE_DEF          ( 60u )
#define     MAX_KVAL_HOLD_VALUE_DEF     ( 20u )
#define     HS_TO_VALUE_DEF             ( 10u )     // holo daviklio taimaut (sekundes)
#define     TRANSMISSION_RATIO_DEF      ( 30u )     // variklio ir rotoriaus diametru santykis
#define     OVH_TIMEOUT_DEF             ( 15u )     // sekundes

#define     USERSET_STEPS_PER_REV_DEF   ( 200u )    //
#define     USERSET_KVAL_RUN_PROC_DEF   ( 24u )     // %
#define     USERSET_KVAL_ACC_PROC_DEF   ( 20u )     // %
#define     USERSET_KVAL_DEC_PROC_DEF   ( 16u )     // %
#define     USERSET_KVAL_HOLD_PROC_DEF  ( 5u )      // %
#define     USERSET_TRES_OCD_MA_DEF     ( 3000u )   // mA
#define     USERSET_TRES_STALL_MA_DEF   ( 2000u )   // mA
#define     USERSET_SPEED_ACC_DEF       ( 100u )    // steps/s^2
#define     USERSET_SPEED_DEC_DEF       ( 100u )    // steps/s^2

#define     SCROLL_RPM_DEF              ( 20u )     // RPM
#define     SCROLL_OFF_CYCLE_TIME_DEF   ( 300u )    // sekundes
#define     SCROLL_ON_CYCLE_TIME_DEF    ( 30u )     // sekundes
#define     SCROLL_SYNC_DEF             ( 0u )


/* kiti defainai */
#define		TESTMODE_KEY				( 0x4949 )
#define     SERVICEMODE_KEY             ( 0x26AA )



/* Beeper */
struct _beeper{
    uint32_t    DataReg;
}Beeper;

extern struct _beeper Beeper;


/* UART */
typedef struct {

    uint8_t         Uart;
    uint8_t         ModbusActive;

    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }MbAddr;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;      // aktyvi reiksme
    }Baudrate;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }Parity;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }StopBits;
    struct{
        uint16_t*   pmbus;      // pointeris i Modbus HR
        uint8_t     cvalue;     // aktyvi reiksme
    }DataBits;
} MbPortParams_TypeDef;

extern MbPortParams_TypeDef MbPortParams;


/*   */
typedef struct{

    //eState_TypeDef                      SMC_State;                  //

    struct{
        uint8_t*                		pId;                        // valdiklio UID (25AA048)
        uint8_t*                		pName;                      // valdiklio pavadinimas
        uint8_t*                		pFWVersion;                   // SW versija
        uint8_t*                        pHWVersion;
        uint8_t*                        pProdCode;
    }StrData;

    struct{
        //const MotorParamSet*  	        pCurrentMotorPreset;        // pointeris i naudojamo variklio parametru preseta
        uint16_t                        Status;                     // variklio busena, nuskaityta is draiverio L6470 per spi ( registras STATUS bitai 5-6 (MOT_STATUS) )
        uint16_t                     	RotSpeedSetting;            // nustatytas sukimosi greitis Normal rezime
        uint8_t                     	RotDirSetting;              // nustatyta sukimosi kriptys Normal rezime
    }MotorData;

    struct{
        uint8_t 			    		Data;                       // is DIP switch nuskaityta reiksme
        struct{
            uint8_t 	        		HallSensor;                 // holo daviklis yra/nera
            uint8_t         		    Scrolling;                  // Scroll rezimas naudojamas/nenaudojamas
            uint8_t 		    		MotorType;                  // naudojamo variklio tipas (0-7)
            eMode_TypeDef 	            ControlMode;                // DIP switch nustatomu opciju reiksme : valdymo rezimas
        }Option;
    }DipSwitch;

    struct{
        struct{
            uint16_t                    adc;
            uint16_t                    mV;
        }VRef;
         struct{
            uint16_t                    adc;
            uint16_t                    mV;
        }Vbus;
        struct{
            uint16_t                    adc;
            uint16_t                    mV;
        }SpReq;
        struct{
            uint16_t                    adc;
            uint16_t                    celsius;
        }McuTemp;
    }ADC_Vals;

}SmcHandle_TypeDef;

extern SmcHandle_TypeDef SMC_Control;


#endif /* DEFS_H_INCLUDED */

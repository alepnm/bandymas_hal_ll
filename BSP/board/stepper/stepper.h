#ifndef STP_H_INCLUDED
#define STP_H_INCLUDED

#include "hardware.h"

/* darbo rezimai */
typedef enum { MODE_MANUAL = 0, MODE_MODBUS, MODE_STEPCLOCK, MODE_TESTMODE } eMode_TypeDef;


#define     LEDS_ON()               LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14|LL_GPIO_PIN_15)
#define     LEDS_OFF()              LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14|LL_GPIO_PIN_15)
#define     LEDS_TOGGLE()           LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_14|LL_GPIO_PIN_15)

#define     LED102_ON()       	    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_15)
#define     LED102_OFF()      	    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15)
#define     LED102_TOGGLE()   	    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_15)

#define     STATUS_LED_ON()         LED102_ON()
#define     STATUS_LED_OFF()        LED102_OFF()
#define     STATUS_LED_TOGGLE()     LED102_TOGGLE()

#define     LED103_ON()       	    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_14)
#define     LED103_OFF()      	    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14)
#define     LED103_TOGGLE()   	    LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_14)

#define     ERROR_LED_ON()          LED103_ON()
#define     ERROR_LED_OFF()         LED103_OFF()
#define     ERROR_LED_TOGGLE()      LED103_TOGGLE()

#define     ALARM_RELAY_ON()        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13)
#define     ALARM_RELAY_OFF()       LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13)

#define     L6470_CS_LOW()          LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11)
#define     L6470_CS_HIGH()         LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11)
#define     L6470_RST_LOW()         LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10)
#define     L6470_RST_HIGH()        LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10)

#define     M25AA_CS_LOW()          LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12)
#define     M25AA_CS_HIGH()         LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12)

#define     HC598_CS_LOW()          LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13)
#define     HC598_CS_HIGH()         LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13)
#define     HC598_LATCH_LOW()       LL_GPIO_ResetOutputPin(GPIOF, LL_GPIO_PIN_6)
#define     HC598_LATCH_HIGH()      LL_GPIO_SetOutputPin(GPIOF, LL_GPIO_PIN_6)
#define     HC598_CTRL_LOW()        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define     HC598_CTRL_HIGH()       LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11)

#define     READ_DI0_INPUT()        LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_0)
#define     READ_DI1_INPUT()        LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_1)
#define     READ_DI2_INPUT()        LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_2)
#define     READ_DI3_INPUT()        LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_14)

#define     READ_REFINT()           STP_ReadADC(LL_ADC_CHANNEL_VREFINT, LL_ADC_RESOLUTION_12B);
#define     READ_VBUS()             STP_ReadADC(LL_ADC_CHANNEL_0, LL_ADC_RESOLUTION_8B);
#define     READ_SPREQ()            STP_ReadADC(LL_ADC_CHANNEL_1, LL_ADC_RESOLUTION_8B);
#define     READ_MCUTEMP()          STP_ReadADC(LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_RESOLUTION_12B);

#define     UNIT_GROUP              0x04
#define     UNIT_SUBGROUP           0x01
#define     UNIT_NAME               "STP"
#define     UNIT_FW_VERSION         "20"
#define     UNIT_HW_VERSION         "30"
#define     UNIT_PROD_CODE          "GRG060"

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





/* Private functions */
void        STP_Start(void);
void        STP_UartSendString(const char* str);
void        STP_ReadADC(uint32_t channel, uint32_t resolution);
void        STP_ReadDipSwitch(void);

//void        BSP_Delay(uint32_t delay);
uint8_t     CheckBaudrateValue(uint32_t baudrate);
uint8_t     CheckBaudrateIndex(uint8_t idx);
uint8_t     GetIndexByBaudrate(uint32_t baudrate);
uint32_t    GetBaudrateByIndex(uint8_t idx);
uint8_t     GetCurrentBaudrateIndex(void);

#endif /* STP_H_INCLUDED */

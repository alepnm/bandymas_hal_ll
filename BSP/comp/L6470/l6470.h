/*

*/

#ifndef L6470_h
#define L6470_h

#include "common.h"


#define PROC_8BIT(x)                (uint8_t)(x * 2.55)         // % -> byte (0 - 0xFF)

// constant definitions for overcurrent thresholds. Write these values to
//  register dSPIN_OCD_TH to set the level at which an overcurrent even occurs.
#define OCD_TH_375mA                0x00
#define OCD_TH_750mA                0x01
#define OCD_TH_1125mA               0x02
#define OCD_TH_1500mA               0x03
#define OCD_TH_1875mA               0x04
#define OCD_TH_2250mA               0x05
#define OCD_TH_2625mA               0x06
#define OCD_TH_3000mA               0x07
#define OCD_TH_3375mA               0x08
#define OCD_TH_3750mA               0x09
#define OCD_TH_4125mA               0x0A
#define OCD_TH_4500mA               0x0B
#define OCD_TH_4875mA               0x0C
#define OCD_TH_5250mA               0x0D
#define OCD_TH_5625mA               0x0E
#define OCD_TH_6000mA               0x0F


// STEP_MODE option values.
// First comes the "microsteps per step" options...
#define STEP_MODE_STEP_SEL          0x07  // Mask for these bits only.
#define STEP_SEL_1                  0x00
#define STEP_SEL_1_2                0x01
#define STEP_SEL_1_4                0x02
#define STEP_SEL_1_8                0x03
#define STEP_SEL_1_16               0x04
#define STEP_SEL_1_32               0x05
#define STEP_SEL_1_64               0x06
#define STEP_SEL_1_128              0x07

// ...next, define the SYNC_EN bit. When set, the BUSYN pin will instead
//  output a clock related to the full-step frequency as defined by the
//  SYNC_SEL bits below.
#define STEP_MODE_SYNC_EN	        0x80  // Mask for this bit
#define SYNC_EN                     (0x01<<7)

// ...last, define the SYNC_SEL modes. The clock output is defined by
//  the full-step frequency and the value in these bits- see the datasheet
//  for a matrix describing that relationship (page 46).
#define STEP_MODE_SYNC_SEL           0x70
#define SYNC_SEL_1_2                (0x00<<4)
#define SYNC_SEL_1                  (0x01<<4)
#define SYNC_SEL_2                  (0x02<<4)
#define SYNC_SEL_4                  (0x03<<4)
#define SYNC_SEL_8                  (0x04<<4)
#define SYNC_SEL_16                 (0x05<<4)
#define SYNC_SEL_32                 (0x06<<4)
#define SYNC_SEL_64                 (0x07<<4)

// Bit names for the ALARM_EN register.
//  Each of these bits defines one potential alarm condition.
//  When one of these conditions occurs and the respective bit in ALARM_EN is set,
//  the FLAG pin will go low. The register must be queried to determine which event
//  caused the alarm.
#define ALARM_EN_OVERCURRENT            0x01
#define ALARM_EN_THERMAL_SHUTDOWN       0x02
#define ALARM_EN_THERMAL_WARNING        0x04
#define ALARM_EN_UNDER_VOLTAGE          0x08
#define ALARM_EN_STALL_DET_A            0x10
#define ALARM_EN_STALL_DET_B            0x20
#define ALARM_EN_SW_TURN_ON             0x40
#define ALARM_EN_WRONG_NPERF_CMD        0x80


#define SPEED_LSPD_OPT                  (0x01<<12)  // Low speed optimization

// CONFIG register renames.

// Oscillator options.
// The dSPIN needs to know what the clock frequency is because it uses that for some
//  calculations during operation.
#define CONFIG_OSC_SEL                  0x000F // Mask for this bit field.
#define CONFIG_INT_16MHZ                0x0000 // Internal 16MHz, no output
#define CONFIG_INT_16MHZ_OSCOUT_2MHZ    0x0008 // Default; internal 16MHz, 2MHz output
#define CONFIG_INT_16MHZ_OSCOUT_4MHZ    0x0009 // Internal 16MHz, 4MHz output
#define CONFIG_INT_16MHZ_OSCOUT_8MHZ    0x000A // Internal 16MHz, 8MHz output
#define CONFIG_INT_16MHZ_OSCOUT_16MHZ   0x000B // Internal 16MHz, 16MHz output
#define CONFIG_EXT_8MHZ_XTAL_DRIVE      0x0004 // External 8MHz crystal
#define CONFIG_EXT_16MHZ_XTAL_DRIVE     0x0005 // External 16MHz crystal
#define CONFIG_EXT_24MHZ_XTAL_DRIVE     0x0006 // External 24MHz crystal
#define CONFIG_EXT_32MHZ_XTAL_DRIVE     0x0007 // External 32MHz crystal
#define CONFIG_EXT_8MHZ_OSCOUT_INVERT   0x000C // External 8MHz crystal, output inverted
#define CONFIG_EXT_16MHZ_OSCOUT_INVERT  0x000D // External 16MHz crystal, output inverted
#define CONFIG_EXT_24MHZ_OSCOUT_INVERT  0x000E // External 24MHz crystal, output inverted
#define CONFIG_EXT_32MHZ_OSCOUT_INVERT  0x000F // External 32MHz crystal, output inverted

// Configure the functionality of the external switch input
#define CONFIG_SW_MODE                  0x0010 // Mask for this bit.
#define CONFIG_SW_HARD_STOP             (0x00<<4) // Default; hard stop motor on switch.
#define CONFIG_SW_USER                  (0x01<<4) // Tie to the GoUntil and ReleaseSW
//  commands to provide jog function.

// Configure the motor voltage compensation mode (see page 34 of datasheet)
#define CONFIG_EN_VSCOMP                0x0020  // Mask for this bit.
#define CONFIG_VS_COMP_DISABLE          (0x00<<5)  // Disable motor voltage compensation.
#define CONFIG_VS_COMP_ENABLE           (0x01<<5)  // Enable motor voltage compensation.

// Configure overcurrent detection event handling
#define CONFIG_OC_SD                    0x0080  // Mask for this bit.
#define CONFIG_OC_SD_DISABLE            (0x00<<7)  // Bridges do NOT shutdown on OC detect
#define CONFIG_OC_SD_ENABLE             (0x01<<7)  // Bridges shutdown on OC detect

// Configure the slew rate of the power bridge output
#define CONFIG_POW_SR                   0x0300  // Mask for this bit field.
#define CONFIG_SR_180V_us               (0x00<<8)  // 180V/us
#define CONFIG_SR_290V_us               (0x02<<8)  // 290V/us
#define CONFIG_SR_530V_us               (0x03<<8)  // 530V/us

// Integer divisors for PWM sinewave generation F_PWM_DEC
#define CONFIG_F_PWM_DEC                0x1C00      // mask for this bit field
#define CONFIG_PWM_MUL_0_625            (0x00<<10)
#define CONFIG_PWM_MUL_0_75             (0x01<<10)
#define CONFIG_PWM_MUL_0_875            (0x02<<10)
#define CONFIG_PWM_MUL_1                (0x03<<10)
#define CONFIG_PWM_MUL_1_25             (0x04<<10)
#define CONFIG_PWM_MUL_1_5              (0x05<<10)
#define CONFIG_PWM_MUL_1_75             (0x06<<10)
#define CONFIG_PWM_MUL_2                (0x07<<10)

// Multiplier for the PWM sinewave frequency F_PWM_INT
#define CONFIG_F_PWM_INT                0xE000     // mask for this bit field.
#define CONFIG_PWM_DIV_1                (0x00<<13)
#define CONFIG_PWM_DIV_2                (0x01<<13)
#define CONFIG_PWM_DIV_3                (0x02<<13)
#define CONFIG_PWM_DIV_4                (0x03<<13)
#define CONFIG_PWM_DIV_5                (0x04<<13)
#define CONFIG_PWM_DIV_6                (0x05<<13)
#define CONFIG_PWM_DIV_7                (0x06<<13)


//typedef uint8_t byte;


typedef enum{
    // Status register bit renames- read-only bits conferring information about the
    //  device to the user.
    STATUS_HIZ                      =   0x0001, // high when bridges are in HiZ mode
    STATUS_BUSY                     =   0x0002, // mirrors BUSY pin
    STATUS_SW_F                     =   0x0004, // low when switch open, high when closed
    STATUS_SW_EVN                   =   0x0008, // active high, set on switch falling edge,
    //  cleared by reading STATUS
    STATUS_DIR                      =   0x0010, // Indicates current motor direction.
    STATUS_NOTPERF_CMD              =   0x0080, // Last command not performed.
    STATUS_WRONG_CMD                =   0x0100, // Last command not valid.
    STATUS_UVLO                     =   0x0200, // Undervoltage lockout is active
    STATUS_TH_WRN                   =   0x0400, // Thermal warning
    STATUS_TH_SD                    =   0x0800, // Thermal shutdown
    STATUS_OCD                      =   0x1000, // Overcurrent detected
    STATUS_STEP_LOSS_A              =   0x2000, // Stall detected on A bridge
    STATUS_STEP_LOSS_B              =   0x4000, // Stall detected on B bridge
    STATUS_SCK_MOD                  =   0x8000, // Step clock mode is active

}L6470_StatusTypeDef;


// Status register motor status field
#define STATUS_MOT_STATUS               0x0060      // field mask
#define STATUS_MOT_STATUS_STOPPED       (0x00<<5) // Motor stopped
#define STATUS_MOT_STATUS_ACCELERATION  (0x01<<5) // Motor accelerating
#define STATUS_MOT_STATUS_DECELERATION  (0x02<<5) // Motor decelerating
#define STATUS_MOT_STATUS_CONST_SPD     (0x03<<5) // Motor at constant speed


// Register address redefines.
//  See the Param_Handler() function for more info about these.
typedef enum{
    REG_ABS_POS     =   ( (uint8_t)0x01 ),
    REG_EL_POS      =   ( (uint8_t)0x02 ),
    REG_MARK        =   ( (uint8_t)0x03 ),
    REG_SPEED       =   ( (uint8_t)0x04 ),
    REG_ACC         =   ( (uint8_t)0x05 ),
    REG_DEC         =   ( (uint8_t)0x06 ),
    REG_MAX_SPEED   =   ( (uint8_t)0x07 ),
    REG_MIN_SPEED   =   ( (uint8_t)0x08 ),
    REG_FS_SPD      =   ( (uint8_t)0x15 ),
    REG_KVAL_HOLD   =   ( (uint8_t)0x09 ),
    REG_KVAL_RUN    =   ( (uint8_t)0x0A ),
    REG_KVAL_ACC    =   ( (uint8_t)0x0B ),
    REG_KVAL_DEC    =   ( (uint8_t)0x0C ),
    REG_INT_SPD     =   ( (uint8_t)0x0D ),
    REG_ST_SLP      =   ( (uint8_t)0x0E ),
    REG_FN_SLP_ACC  =   ( (uint8_t)0x0F ),
    REG_FN_SLP_DEC  =   ( (uint8_t)0x10 ),
    REG_K_THERM     =   ( (uint8_t)0x11 ),
    REG_ADC_OUT     =   ( (uint8_t)0x12 ),
    REG_OCD_TH      =   ( (uint8_t)0x13 ),
    REG_STALL_TH    =   ( (uint8_t)0x14 ),
    REG_STEP_MODE   =   ( (uint8_t)0x16 ),
    REG_ALARM_EN    =   ( (uint8_t)0x17 ),
    REG_CONFIG      =   ( (uint8_t)0x18 ),
    REG_STATUS      =   ( (uint8_t)0x19 )
}L6470_Registers_TypeDef;

//dSPIN commands
typedef enum{
    CMD_NOP         =   ( (uint8_t)0x00 ),
    CMD_SET_PARAM   =   ( (uint8_t)0x00 ),
    CMD_GET_PARAM   =   ( (uint8_t)0x20 ),
    CMD_RUN         =   ( (uint8_t)0x50 ),
    CMD_STEP_CLOCK  =   ( (uint8_t)0x58 ),
    CMD_MOVE        =   ( (uint8_t)0x40 ),
    CMD_GOTO        =   ( (uint8_t)0x60 ),
    CMD_GOTO_DIR    =   ( (uint8_t)0x68 ),
    CMD_GO_UNTIL    =   ( (uint8_t)0x82 ),
    CMD_RELEASE_SW  =   ( (uint8_t)0x92 ),
    CMD_GO_HOME     =   ( (uint8_t)0x70 ),
    CMD_GO_MARK     =   ( (uint8_t)0x78 ),
    CMD_RESET_POS   =   ( (uint8_t)0xD8 ),
    CMD_RESET_DEVICE =  ( (uint8_t)0xC0 ),
    CMD_SOFT_STOP   =   ( (uint8_t)0xB0 ),
    CMD_HARD_STOP   =   ( (uint8_t)0xB8 ),
    CMD_SOFT_HIZ    =   ( (uint8_t)0xA0 ),
    CMD_HARD_HIZ    =   ( (uint8_t)0xA8 ),
    CMD_GET_STATUS  =   ( (uint8_t)0xD0 )
}L6470_Commands_TypeDef;

/* direction options */
#define FWD  0x01
#define REV  0x00

/* action options */
#define ACTION_RESET            0x00
#define ACTION_COPY             0x01



/* Varikliu presetai */
typedef struct{
    uint8_t ID;                 // preseto ID
    uint8_t StepsPerRev;        // variklio stepu per apsisukima
    struct{
        uint8_t RunValue;           // KVAL_RUN reiksme, %
        uint8_t AccValue;           // KVAL_ACC reiksme, %
        uint8_t DecValue;           // KVAL_DEC reiksme, %
        uint8_t HoldValue;          // KVAL_HOLD reiksme, %
    }Kval;
    struct{
        uint16_t OcdValue;          // OCD_TH reiksme, mA
        uint16_t StallValue;        // STALL_TH reiksme, mA
    }Treshold;
    struct{
        uint16_t Acceleration;      //
        uint16_t Deceleration;      //
    }Speed;
}MotorParamSet;


/* public functions */
void L6470_Init(void);

void L6470_setMicroSteps(int microSteps);
void L6470_setCurrent(int current);
void L6470_setMaxSpeed(int speed);
void L6470_setMinSpeed(int speed);
void L6470_setAcc(float acceleration);
void L6470_setDec(float deceleration);
void L6470_setOverCurrent(float ma_current);
void L6470_setStallCurrent(float ma_current);
void L6470_setThresholdSpeed(float threshold);

void L6470_SetLowSpeedOpt(bool enable);

void L6470_run(byte dir, float spd);
void L6470_StepClock(byte dir);

void L6470_goHome(void);
void L6470_setAsHome(void);

void L6470_goMark(void);
void L6470_move(long n_step);
void L6470_goTo(long pos);
void L6470_goTo_DIR(byte dir, long pos);
void L6470_goUntil(byte act, byte dir, unsigned long spd);

bool L6470_isBusy(void);

void L6470_releaseSW(byte act, byte dir);

float L6470_getSpeed(void);
long L6470_getPos(void);
void L6470_setMark_currPos( void );
void L6470_setMark(long value);


void L6470_resetPos(void);
void L6470_resetDev(void);
void L6470_softStop(void);
void L6470_hardStop(void);
void L6470_softFree(void);
void L6470_hardFree(void);
int L6470_getStatus(void);

void SetParam(byte param, unsigned long value);
unsigned long GetParam(byte param);

/* konvertoriai */
float ConvertRpmToStepsPerSec( float rpm );
float ConvertStepsPerSecToRpm( float steps_per_sec );

unsigned long AccCalc(float stepsPerSecPerSec);
unsigned long DecCalc(float stepsPerSecPerSec);
unsigned long MaxSpdCalc(float stepsPerSec);
unsigned long MinSpdCalc(float stepsPerSec);
unsigned long FSCalc(float stepsPerSec);
unsigned long IntSpdCalc(float stepsPerSec);
unsigned long SpdCalc(float stepsPerSec);

const MotorParamSet* GetPresetByID(uint8_t preset_id);

float SpeedCompensationCalc(float set);

#endif

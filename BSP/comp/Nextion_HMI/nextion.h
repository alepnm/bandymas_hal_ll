#ifndef NEXTION_H_INCLUDED
#define NEXTION_H_INCLUDED

#include "defs.h"

#define NEX_INVALID_INSTRUCTION         0x00
#define NEX_SUCCESSFUL_EXECUTE          0x01
#define NEX_COMPONENTID_INVALID         0x02
#define NEX_PAGEID_INVALID              0x03
#define NEX_PICTUREID_INVALID           0x04
#define NEX_FONTID_INVALID              0x05
#define NEX_BAUDERATE_INVALID           0x11
#define NEX_CURVE_CONTROL_ID_INVALID    0x12
#define NEX_VARIABLE_NAME_INVALID       0x1A
#define NEX_VARIABLE_OPERATION_INVALID  0x1B
#define NEX_FILED_TO_ASSIGN             0x1C
#define NEX_OPERATE_EEPROM_FILED        0x1D
#define NEX_PARAMETER_QUANTITY_INVALID  0x1E
#define NEX_IO_OPERATION_FILED          0x1F
#define NEX_UNDEF_ESCAPE_CHARACTERS     0x20
#define NEX_TOO_LONG_VAR_NAME           0x23

#define NEX_TOUCH_EVENT_DATA            0x65
#define NEX_CURRENT_PAGEID_NUMBER       0x66
#define NEX_TOUCH_COORDINATE            0x67
#define NEX_TOUCH_EVENT_IN_SLEEP_MODE   0x68
#define NEX_STRING_VARIABLE_DATA        0x70
#define NEX_NUMERIC_VARIABLE_DATA       0x71
#define NEX_DEV_IN_SLEEP_MODE           0x86
#define NEX_DEV_WAKEUP                  0x87
#define NEX_SYSTEM_SUCCESS_START        0x88
#define NEX_START_SDCARD_UPGRADE        0x89
#define NEX_DATA_TRANSMIT_FINISHED      0xFD
#define NEX_DATA_TRANSMIT_READY         0xFE

#define NEX_COLOR_RED                   0xF800
#define NEX_COLOR_BLUE                  0x001F
#define NEX_COLOR_GREY                  0x8430
#define NEX_COLOR_BLACK                 0x0000
#define NEX_COLOR_WHITE                 0xFFFF
#define NEX_COLOR_GREEN                 0x07E0
#define NEX_COLOR_BROWN                 0xBC40
#define NEX_COLOR_YELLOW                0xFFE0



void Nextion_DataProcess(uint8_t* buf);
void Nextion_CommandPrepare(void);



#endif /* NEXTION_H_INCLUDED */



#include "nextion.h"


const char cmd_dim[3]       = "dim";
const char cmd_dims[4]      = "dims";
const char cmd_baud[4]      = "baud";
const char cmd_bauds[5]     = "bauds";
const char cmd_spax[4]      = "spax";
const char cmd_spay[4]      = "spay";
const char cmd_thc[3]       = "thc";
const char cmd_thdra[5]     = "thdra";
const char cmd_ussp[4]      = "ussp";
const char cmd_thsp[4]      = "thsp";
const char cmd_thup[4]      = "thup";
const char cmd_sendxy[6]    = "sendxy";
const char cmd_delay[5]     = "delay";
const char cmd_sleep[5]     = "sleep";
const char cmd_bkcmd[5]     = "bkcmd";
const char cmd_rand[4]      = "rand";
const char cmd_sys0[4]      = "sys0", cmd_sys1[4] = "sys1", cmd_sys2[4] = "sys2";
const char cmd_rtc0[4]      = "rtc0", cmd_rtc1[4] = "rtc1", cmd_rtc2[4] = "rtc2", cmd_rtc3[4] = "rtc3", cmd_rtc4[4] = "rtc4", cmd_rtc5[4] = "rtc5", cmd_rtc6[4] = "rtc6";                       //Enhanced
const char cmd_pio0[4]      = "pio0", cmd_pio1[4] = "pio1", cmd_pio2[4] = "pio2", cmd_pio3[4] = "pio3", cmd_pio4[4] = "pio4", cmd_pio5[4] = "pio5", cmd_pio6[4] = "pio6", cmd_pio7[4] = "pio7"; //Enhanced
const char cmd_pwm4[4]      = "pwm4", cmd_pwm5[4] = "pwm5", cmd_pwm6[4] = "pwm6";   //Enhanced
const char cmd_pwmf[4]      = "pwmf";   //Enhanced
const char cmd_wup[3]       = "wup";



/* duomenu is HMI tvarkymas */
void Nextion_DataProcess(uint8_t* buf){




}

/* komandos i HMI paruosimas */
void Nextion_CommandPrepare(void){




}

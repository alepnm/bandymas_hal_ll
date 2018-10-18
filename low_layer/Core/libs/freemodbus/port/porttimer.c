/*
 * FreeModbus Libary: LPC214X Port
 * Copyright (C) 2007 Tiago Prado Lone <tiago@maxwellbohr.com.br>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2007/04/24 23:15:18 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* Statics */
static USHORT usT35TimeOut50us;

/* ----------------------- static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTimeOut50us )
{

    LL_TIM_InitTypeDef TIM_InitStruct;

    /* backup T35 ticks */
    usT35TimeOut50us = usTimeOut50us;

    TIM_InitStruct.Prescaler = (uint32_t)(SystemCoreClock / 48000) - 1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = (uint32_t)( usTimeOut50us - 1 );
    LL_TIM_Init(TIM6, &TIM_InitStruct);

    //LL_TIM_EnableIT_UPDATE(TIM6);

    return TRUE;
}


void vMBPortTimersEnable( )
{
    LL_TIM_SetCounter(TIM6, 0);
    LL_TIM_EnableCounter(TIM6);
    LL_TIM_EnableIT_UPDATE(TIM6);
}

void vMBPortTimersDisable( )
{
    LL_TIM_DisableCounter(TIM6);
    LL_TIM_DisableIT_UPDATE(TIM6);
}


/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
//static void prvvTIMERExpiredISR( void ) {
//
//    ( void )pxMBPortCBTimerExpired( );
//
//}

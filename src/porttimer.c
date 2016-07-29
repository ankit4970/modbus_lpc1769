/**
 * @file 				porttimer.c
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
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
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
//#include "mbed.h"           // Cam

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "timer.h"
/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );
USHORT gusTimer3_5 ;
#define SBIT_TIMER0 	1
#define SBIT_TIMER1		2
#define CCLK			100000000L
#define PCLK			CCLK

/**
 -----------------------------------------------------------------------------------------------------------------------
 xMBPortTimersInit
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
BOOL xMBPortTimersInit( USHORT usTimer3_5 )
{
	/* Timer 0*/
	gusTimer3_5 = usTimer3_5;
	printf("Initializing timer\n");
    LPC_SC->PCONP |= (1<<SBIT_TIMER0);

    LPC_TIM0->PR = 0;                   	// Prscaler Register = 0
    LPC_TIM0->PC = 0;
    LPC_TIM0->TC = 0;

    LPC_TIM0->MR0 = usTimer3_5;    	// Interval of (50us * usTim1Timerout50us)
    LPC_TIM0->MCR = 3;                  	// Bit 0 = 1 - Interruption on MR0 // Bit 1 = 1 - Reset on MR0

    LPC_TIM0->CTCR = 0;
    LPC_TIM0->TCR  = 0x02;
    LPC_TIM0->TCR  = 0x01;

#if 0
    /* Timer 1*/
    LPC_SC->PCONP |= (1<<SBIT_TIMER1);

    LPC_TIM1->PR = 0;                   	// Prscaler Register = 0
    LPC_TIM1->PC = 0;
    LPC_TIM1->TC = 0;

	LPC_TIM1->MR0 = usTimer3_5;    	// Interval of (50us * usTim1Timerout50us)
	LPC_TIM1->MCR = 3;                  	// Bit 0 = 1 - Interruption on MR0 // Bit 1 = 1 - Reset on MR0

	LPC_TIM1->CTCR  = 0;
	LPC_TIM1->TCR  = 0x02;
	LPC_TIM1->TCR  = 0x01;

	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
#endif

	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);

	return TRUE;
}



/**
 -----------------------------------------------------------------------------------------------------------------------
 vMBPortTimersEnable
 -----------------------------------------------------------------------------------------------------------------------
*   Enable timer
*
* 	@date       			JUN/29/2016
* 	@author                 Ankit
* 	@pre                    None
* 	@return	 				None
************************************************************************************************************************
*/
void vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
    // Cam - firstly detach from any existing timeout
    //toMBUS.detach();
    // Cam - now attach the timeout to the prvvTIMERExpiredISR routine    
    //toMBUS.attach_us(&prvvTIMERExpiredISR, usInterval);
	LPC_TIM0->MR0 = gusTimer3_5;
	LPC_TIM0->TCR = 0x02;
	LPC_TIM0->TCR = 0x01;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 vMBPortTimersDisable
-----------------------------------------------------------------------------------------------------------------------
*   Disable timer
*
* 	@date       			JUN/29/2016
* 	@author                 Ankit
* 	@pre                    None
* 	@return	 				None
************************************************************************************************************************
*/
void vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
	LPC_TIM0->TCR = 0x02;
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
#if 1
/**
 -----------------------------------------------------------------------------------------------------------------------
 prvvTIMERExpiredISR
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
static void prvvTIMERExpiredISR( void )
{
    (void)pxMBPortCBTimerExpired( );
    // Cam - disable further interrupts by detaching
    //toMBUS.detach();
}
#endif
/**
 -----------------------------------------------------------------------------------------------------------------------
 TIMER0_IRQHandler
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
void TIMER0_IRQHandler(void)
{
	( void )prvvTIMERExpiredISR( );
	LPC_TIM0->IR = 0xFF;
}


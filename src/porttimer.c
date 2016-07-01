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

#define SBIT_TIMER0 1


/* ----------------------- System Variables ---------------------------------*/
//Timeout toMBUS;             // Cam - mbed timeout
static ULONG usInterval;    // Cam - timeout interval in microseconds 



/* ----------------------- Start implementation -----------------------------*/
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
BOOL xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    usInterval = 50 * usTim1Timerout50us;

    LPC_SC->PCONP |= (1<<SBIT_TIMER0);

	// Timer0 Configuration
    LPC_TIM0->PR = 0;                   // Prscaler Register = 0
    LPC_TIM0->PC = 0;                   // Prscaler Counter = 0

    LPC_TIM0->TC = 0;                   // Timer Counter = 0

    //LPC_TIM0->MR0 = ( PCLK / 20000 ) * usTim1Timerout50us;      // Interval of (50us * usTim1Timerout50us)
    LPC_TIM0->MCR = 3;                  // Bit 0 = 1 - Interruption on MR0
	// Bit 1 = 1 - Reset on MR0

    LPC_TIM0->TCR = 0x02;                  // Timer Counter and Prescale Counter Disabled

    LPC_TIM0->TCR = 0x01;
	// Configure Timer0 Interruption
	//VICVectAddr1 = ( unsigned int )prvvTIMERExpiredISR; // Timer0 Interruption - Priority 1
	//VICVectCntl1 = 0x20 | 4;
	//VICIntEnable = ( 1 << 4 );  // Enable Timer0 Interruption

	//return TRUE;

    /* Enable timer interrupt */
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);

	return TRUE;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 vMBPortTimersEnable
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
void vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
    // Cam - firstly detach from any existing timeout
    //toMBUS.detach();
    // Cam - now attach the timeout to the prvvTIMERExpiredISR routine    
    //toMBUS.attach_us(&prvvTIMERExpiredISR, usInterval);

	LPC_TIM0->TCR = 0x02;
	LPC_TIM0->TCR = 0x01;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 vMBPortTimersDisable
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
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
#if 0
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
	( void )pxMBPortCBTimerExpired( );

	LPC_TIM0->IR = 0xFF;
}


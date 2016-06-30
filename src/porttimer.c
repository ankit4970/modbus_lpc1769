/*
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

/* ----------------------- System Variables ---------------------------------*/
//Timeout toMBUS;             // Cam - mbed timeout
static ULONG usInterval;    // Cam - timeout interval in microseconds 

/**
 * @brief	Enables a match interrupt that fires when the terminal count
 *			matches the match counter value.
 * @param	pTMR		: Pointer to timer IP register address
 * @param	matchnum	: Match timer, 0 to 3
 * @return	Nothing
 */
static inline void Chip_TIMER_MatchEnableInt(LPC_TIM_TypeDef *pTMR, int8_t matchnum)
{
	pTMR->MCR |= TIMER_INT_ON_MATCH(matchnum);
}
/**
 * @brief	Clears a (pending) match interrupt
 * @param	pTMR		: Pointer to timer IP register address
 * @param	matchnum	: Match interrupt number to clear
 * @return	Nothing
 * @note	Clears a pending timer match interrupt.
 */
static inline void Chip_TIMER_ClearMatch(LPC_TIM_TypeDef *pTMR, int8_t matchnum)
{
	pTMR->IR = TIMER_IR_CLR(matchnum);
}

/**
 * @brief	Determine if a match interrupt is pending
 * @param	pTMR		: Pointer to timer IP register address
 * @param	matchnum	: Match interrupt number to check
 * @return	false if the interrupt is not pending, otherwise true
 * @note	Determine if the match interrupt for the passed timer and match
 * counter is pending.
 */
static inline BOOL Chip_TIMER_MatchPending(LPC_TIM_TypeDef *pTMR, int8_t matchnum)
{
	return (BOOL) ((pTMR->IR & TIMER_MATCH_INT(matchnum)) != 0);
}


/**
 * @brief	Sets a timer match value
 * @param	pTMR		: Pointer to timer IP register address
 * @param	matchnum	: Match timer to set match count for
 * @param	matchval	: Match value for the selected match count
 * @return	Nothing
 * @note	Sets one of the timer match values.
 */
static inline void Chip_TIMER_SetMatch(LPC_TIM_TypeDef *pTMR, int8_t matchnum, uint32_t matchval)
{
	pTMR->MR0 = matchval;
}

/* Resets the timer terminal and prescale counts to 0 */
void Chip_TIMER_Reset(LPC_TIM_TypeDef *pTMR)
{
	uint32_t reg;

	/* Disable timer, set terminal count to non-0 */
	reg = pTMR->TCR;
	pTMR->TCR = 0;
	pTMR->TC = 1;

	/* Reset timer counter */
	pTMR->TCR = TIMER_RESET;

	/* Wait for terminal count to clear */
	while (pTMR->TC != 0) {}

	/* Restore timer state */
	pTMR->TCR = reg;
}
/**
 * @brief	For the specific match counter, enables reset of the terminal count register when a match occurs
 * @param	pTMR		: Pointer to timer IP register address
 * @param	matchnum	: Match timer, 0 to 3
 * @return	Nothing
 */
static inline void Chip_TIMER_ResetOnMatchEnable(LPC_TIM_TypeDef *pTMR, int8_t matchnum)
{
	pTMR->MCR |= TIMER_RESET_ON_MATCH(matchnum);
}

/**
 * @brief	Enables the timer (starts count)
 * @param	pTMR	: Pointer to timer IP register address
 * @return	Nothing
 * @note	Enables the timer to start counting.
 */
static inline void Chip_TIMER_Enable(LPC_TIM_TypeDef *pTMR)
{
	pTMR->TCR |= TIMER_ENABLE;
}


/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit( USHORT usTim1Timerout50us )
{
    usInterval = 50 * usTim1Timerout50us;
    return TRUE;

    /* Enable timer interrupt */
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);
}


void vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
    
    // Cam - firstly detach from any existing timeout
    //toMBUS.detach();
    // Cam - now attach the timeout to the prvvTIMERExpiredISR routine    
    //toMBUS.attach_us(&prvvTIMERExpiredISR, usInterval);

	LPC_TIM0->TCR |= TIMER_ENABLE;
}


void vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
    
    // Cam - disable further interrupts by detaching
    //toMBUS.detach();
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
    // Cam - disable further interrupts by detaching
    //toMBUS.detach();
}

void TIMER0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIM0, 1))
	{
		//Chip_TIMER_ClearMatch(LPC_TIM0, 1);
		( void )pxMBPortCBTimerExpired( );
	}
}


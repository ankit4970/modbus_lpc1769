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
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
//#include "mbed.h"                   // Cam

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"


/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );
static void prvvUARTISR( void );

/* ----------------------- System Variables ---------------------------------*/
//Serial pc(USBTX, USBRX);            // Cam - mbed USB serial port

//Ticker simISR;                      // Cam - mbed ticker
                                    // we don't have the TX buff empty interrupt, so
                                    // we just interrupt every 1 mSec and read RX & TX
                                    // status to simulate the proper ISRs.

static BOOL RxEnable, TxEnable;     // Cam - keep a static copy of the RxEnable and TxEnable
                                    // status for the simulated ISR (ticker)


/* ----------------------- Start implementation -----------------------------*/
// Cam - This is called every 1mS to simulate Rx character received ISR and
// Tx buffer empty ISR.
static void prvvUARTISR( void )
{
    if (TxEnable)
    {
        //if(pc.writeable())
        {
            prvvUARTTxReadyISR();
        }
    }
    if (RxEnable)
    {
        //if(pc.readable())
        {
            prvvUARTRxISR();
        }
    }
}

void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
    RxEnable = xRxEnable;
    TxEnable = xTxEnable;
}

BOOL xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    //simISR.attach_us(&prvvUARTISR,1000);    // Cam - attach prvvUARTISR to a 1mS ticker to simulate serial interrupt behaviour
                                            // 1mS is just short of a character time at 9600 bps, so quick enough to pick
                                            // up status on a character by character basis.

	//const uint32_t  Baud_Rate = 9600;

	// Step 1: UART2 power/clock control bit(24) of PCONP
	LPC_SC->PCONP &=~ (1<<24);
	LPC_SC->PCONP |= (1<<24);

	// Step 2: PCLKSEL1 bit 16 and 17 of Peripheral Clock Selection register1 for Uart2, controls the rate of the clock signal
	// that will be supplied to the corresponding peripheral Function peripheral clock selection
	// 00 CCLK/4
	// 01 CCLK
	// 10 CCLK/4
	// 11 CLK/4
	LPC_SC->PCLKSEL1 &=~(3<<16);
	LPC_SC->PCLKSEL1 |= (1<<16);

	// Step 3:// table 280
	LPC_UART2->LCR &=~ (3<<0 | 1<<7);

	// LCR bit o and 1, Word length select
	LPC_UART2->LCR |= (3<<0);						//00 5-bit character length
													//01 6-bit character length
													//10 7-bit character length
													//11 8-bit character length

	//Enable, Divisor latch access bit(DLAB)
	LPC_UART2->LCR |= (1<<7);
	LPC_UART2->DLM = 0x00;
	//LPC_UART3->DLL =0x38;
	LPC_UART2->DLL =( sys_get_cpu_clock() / (16 * ulBaudRate));

	// Step 4: FIFO not use:
	LPC_PINCON->PINSEL0 &=~ (3<<20 | 3<<22);
	LPC_PINCON->PINSEL0 |= (1<<20 | 1<<22);    //TXD3 | RXD3
	LPC_PINCON->PINMODE4 &=~ (3<<20 | 3<<22);
	LPC_PINCON->PINMODE4 |= (3<<22);           // 11 pull down

	// Step 6: Disable, Divisor latch access bit(DLAB)
	LPC_UART2->LCR &=~ (1<<7);
	NVIC_EnableIRQ(UART2_IRQn);

	//enable receive data available interrupt for uart2
	LPC_UART2->IER &=~ (1<<0);
	LPC_UART2->IER |= (1<<0);

    return TRUE;
}

BOOL xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    //pc.putc( ucByte);

	LPC_UART2->THR = ucByte;
	while( !(LPC_UART2->LSR & (1 << 5)));
    return TRUE;
}

BOOL xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	//* pucByte = pc.getc();

	*pucByte = LPC_UART2->RBR;
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR(void)
{
    pxMBFrameCBTransmitterEmpty();
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR(void)
{
    pxMBFrameCBByteReceived( );
}



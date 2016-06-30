/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbutils.c,v 1.6 2007/02/18 23:49:07 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbproto.h"
#include "LPC17xx.h"
/* ----------------------- Defines ------------------------------------------*/
#define BITS_UCHAR      8U

#define INTERNAL_CLOCK		    (4  * 1000 * 1000UL)    ///< Do not change, this is the same on all LPC17XX
#define EXTERNAL_CLOCK          (12 * 1000 * 1000UL)    ///< Change according to your board specification
#define RTC_CLOCK               (32768UL)               ///< Do not change, this is the typical RTC crystal value
#define configTICK_RATE_HZ		( 1000 )
#define OS_MS(x)						( x / MS_PER_TICK() )        ///< Ticks to millisecond conversion
#define OS_MS(x)						( x / MS_PER_TICK() )        ///< Ticks to millisecond conversion

void delay(uint32_t delayInMs)
{
	LPC_TIM0->TCR = 0x02;		/* reset timer */
	LPC_TIM0->PR  = 0x00;		/* set prescaler to zero */
	LPC_TIM0->MR0 = delayInMs * (9000000 / 1000-1);
	LPC_TIM0->IR  = 0xff;		/* reset all interrrupts */
	LPC_TIM0->MCR = 0x04;		/* stop timer on match */
	LPC_TIM0->TCR = 0x01;		/* start timer */

	/* wait until delay time has elapsed */
	while (LPC_TIM0->TCR & 0x01);

	return;
}

unsigned int sys_get_cpu_clock()
{
	unsigned clock = 0;

	/* Determine clock frequency according to clock register values             */
	if (((LPC_SC->PLL0STAT >> 24) & 3) == 3)
	{ /* If PLL0 enabled and connected */
	    switch (LPC_SC->CLKSRCSEL & 0x03)
	    {
	        case 0: /* Int. RC oscillator => PLL0    */
	        case 3: /* Reserved, default to Int. RC  */
	            clock = (INTERNAL_CLOCK
	                    * ((2 * ((LPC_SC->PLL0STAT & 0x7FFF) + 1)))
	                    / (((LPC_SC->PLL0STAT >> 16) & 0xFF) + 1)
	                    / ((LPC_SC->CCLKCFG & 0xFF) + 1));
	            break;

	        case 1: /* Main oscillator => PLL0       */
	            clock = (EXTERNAL_CLOCK
	                    * ((2 * ((LPC_SC->PLL0STAT & 0x7FFF) + 1)))
	                    / (((LPC_SC->PLL0STAT >> 16) & 0xFF) + 1)
	                    / ((LPC_SC->CCLKCFG & 0xFF) + 1));
	            break;

	        case 2: /* RTC oscillator => PLL0        */
	            clock = (RTC_CLOCK
	                    * ((2 * ((LPC_SC->PLL0STAT & 0x7FFF) + 1)))
	                    / (((LPC_SC->PLL0STAT >> 16) & 0xFF) + 1)
	                    / ((LPC_SC->CCLKCFG & 0xFF) + 1));
	            break;
	    }
	}
	else
	{
	    switch (LPC_SC->CLKSRCSEL & 0x03)
	    {
	        case 0: /* Int. RC oscillator => PLL0    */
	        case 3: /* Reserved, default to Int. RC  */
	            clock = INTERNAL_CLOCK / ((LPC_SC->CCLKCFG & 0xFF) + 1);
	            break;
	        case 1: /* Main oscillator => PLL0       */
	            clock = EXTERNAL_CLOCK / ((LPC_SC->CCLKCFG & 0xFF) + 1);
	            break;
	        case 2: /* RTC oscillator => PLL0        */
	            clock = RTC_CLOCK / ((LPC_SC->CCLKCFG & 0xFF) + 1);
	            break;
	    }
	}

	return clock;
}


/* ----------------------- Start implementation -----------------------------*/
void
xMBUtilSetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits,
                UCHAR ucValue )
{
    USHORT          usWordBuf;
    USHORT          usMask;
    USHORT          usByteOffset;
    USHORT          usNPreBits;
    USHORT          usValue = ucValue;

    assert( ucNBits <= 8 );
    assert( ( size_t )BITS_UCHAR == sizeof( UCHAR ) * 8 );

    /* Calculate byte offset for first byte containing the bit values starting
     * at usBitOffset. */
    usByteOffset = ( USHORT )( ( usBitOffset ) / BITS_UCHAR );

    /* How many bits precede our bits to set. */
    usNPreBits = ( USHORT )( usBitOffset - usByteOffset * BITS_UCHAR );

    /* Move bit field into position over bits to set */
    usValue <<= usNPreBits;

    /* Prepare a mask for setting the new bits. */
    usMask = ( USHORT )( ( 1 << ( USHORT ) ucNBits ) - 1 );
    usMask <<= usBitOffset - usByteOffset * BITS_UCHAR;

    /* copy bits into temporary storage. */
    usWordBuf = ucByteBuf[usByteOffset];
    usWordBuf |= ucByteBuf[usByteOffset + 1] << BITS_UCHAR;

    /* Zero out bit field bits and then or value bits into them. */
    usWordBuf = ( USHORT )( ( usWordBuf & ( ~usMask ) ) | usValue );

    /* move bits back into storage */
    ucByteBuf[usByteOffset] = ( UCHAR )( usWordBuf & 0xFF );
    ucByteBuf[usByteOffset + 1] = ( UCHAR )( usWordBuf >> BITS_UCHAR );
}

UCHAR
xMBUtilGetBits( UCHAR * ucByteBuf, USHORT usBitOffset, UCHAR ucNBits )
{
    USHORT          usWordBuf;
    USHORT          usMask;
    USHORT          usByteOffset;
    USHORT          usNPreBits;

    /* Calculate byte offset for first byte containing the bit values starting
     * at usBitOffset. */
    usByteOffset = ( USHORT )( ( usBitOffset ) / BITS_UCHAR );

    /* How many bits precede our bits to set. */
    usNPreBits = ( USHORT )( usBitOffset - usByteOffset * BITS_UCHAR );

    /* Prepare a mask for setting the new bits. */
    usMask = ( USHORT )( ( 1 << ( USHORT ) ucNBits ) - 1 );

    /* copy bits into temporary storage. */
    usWordBuf = ucByteBuf[usByteOffset];
    usWordBuf |= ucByteBuf[usByteOffset + 1] << BITS_UCHAR;

    /* throw away unneeded bits. */
    usWordBuf >>= usNPreBits;

    /* mask away bits above the requested bitfield. */
    usWordBuf &= usMask;

    return ( UCHAR ) usWordBuf;
}

eMBException
prveMBError2Exception( eMBErrorCode eErrorCode )
{
    eMBException    eStatus;

    switch ( eErrorCode )
    {
        case MB_ENOERR:
            eStatus = MB_EX_NONE;
            break;

        case MB_ENOREG:
            eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
            break;

        case MB_ETIMEDOUT:
            eStatus = MB_EX_SLAVE_BUSY;
            break;

        default:
            eStatus = MB_EX_SLAVE_DEVICE_FAILURE;
            break;
    }

    return eStatus;
}

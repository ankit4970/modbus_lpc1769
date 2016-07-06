/**
 * @file 				mbrtu.c
 *
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
 * File: $Id: mbrtu.c,v 1.18 2007/09/12 10:15:56 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbrtu.h"
#include "mbframe.h"

#include "mbcrc.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256     /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

/* ----------------------- Type definitions ---------------------------------*/
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;

typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT               /*!< Transmitter is in transfer state. */
} eMBSndState;

/* ----------------------- Static variables ---------------------------------*/
static volatile eMBSndState eSndState;
static volatile eMBRcvState eRcvState;

volatile UCHAR  ucRTUBuf[MB_SER_PDU_SIZE_MAX];

static volatile UCHAR *pucSndBufferCur;
static volatile USHORT usSndBufferCount;

static volatile USHORT usRcvBufferPos;

/* ----------------------- Start implementation -----------------------------*/
/**
 -----------------------------------------------------------------------------------------------------------------------
 eMBRTUInit
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
eMBErrorCode eMBRTUInit( UCHAR ucSlaveAddress, UCHAR ucPort, ULONG ulBaudRate, eMBParity eParity )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    ULONG           usTimer1_5=0;
    ULONG 			usTimer3_5=0;

    printf("Initializing RTU\n");
    ( void )ucSlaveAddress;
    ENTER_CRITICAL_SECTION(  );

    /* Modbus RTU uses 8 Databits. */
    if( xMBPortSerialInit( ucPort, ulBaudRate, 8, eParity ) != TRUE )
    {
        printf("Serial port initialization failed\n");
    	eStatus = MB_EPORTERR;
    }
    else
    {
        /* If baudrate > 19200 then we should use the fixed timer values
         * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
         */
        if( ulBaudRate > 19200 )
        {
        	usTimer1_5 = 75000;       	/* 750 us */
            usTimer3_5 = 175000;		/* 1.750 ms*/
        }
        else
        {
            /* The timer reload value for a character is given by:
             *
             * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
             *             = 11 * Ticks_per_1s / Baudrate
             *             = 220000 / Baudrate
             * The reload for t3.5 is 1.5 times this value and similary
             * for t3.5.
             */
        	usTimer1_5 = (1500000/ulBaudRate); 		//
        	usTimer3_5 = (3500000/ulBaudRate);
        }
        printf("Initialize timer");
        if( xMBPortTimersInit( ( USHORT ) usTimer3_5 ) != TRUE )
        {
            eStatus = MB_EPORTERR;
        }
    }

    EXIT_CRITICAL_SECTION(  );

    return eStatus;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 eMBRTUStart
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
void eMBRTUStart( void )
{
    ENTER_CRITICAL_SECTION();

    /* Initially the receiver is in the state STATE_RX_INIT. we start
     * the timer and if no character is received within t3.5 we change
     * to STATE_RX_IDLE. This makes sure that we delay startup of the
     * modbus protocol stack until the bus is free.
     */
    eRcvState = STATE_RX_INIT;
    vMBPortSerialEnable(TRUE, FALSE);
    vMBPortTimersEnable();

    EXIT_CRITICAL_SECTION(  );
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 eMBRTUStop
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
void eMBRTUStop( void )
{
    ENTER_CRITICAL_SECTION(  );
    vMBPortSerialEnable( FALSE, FALSE );
    vMBPortTimersDisable(  );
    EXIT_CRITICAL_SECTION(  );
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 eMBRTUReceive
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
eMBErrorCode eMBRTUReceive( UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{
    BOOL            xFrameReceived = FALSE;
    eMBErrorCode    eStatus = MB_ENOERR;
    int i =0;
    ENTER_CRITICAL_SECTION(  );
    assert( usRcvBufferPos < MB_SER_PDU_SIZE_MAX );
        
    /*for(i=0;i<8;i++)
    {
    	printf("Received data %x\n",ucRTUBuf[i]);
    }*/

    /* Length and CRC check */
    if( ( usRcvBufferPos >= MB_SER_PDU_SIZE_MIN )
        && ( usMBCRC16( ( UCHAR * ) ucRTUBuf, usRcvBufferPos ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layer
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = ucRTUBuf[MB_SER_PDU_ADDR_OFF];

        /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
         * size of address field and CRC checksum.
         */
        *pusLength = ( USHORT )( usRcvBufferPos - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

        /* Return the start of the Modbus PDU to the caller. */
        *pucFrame = ( UCHAR * ) & ucRTUBuf[MB_SER_PDU_PDU_OFF];
        xFrameReceived = TRUE;
        
        // Added by Cam
        // Now that the poll routine knows about the received frame, 
        // clear the receive buffer position ready for the next frame received
        usRcvBufferPos = 0;

    }
    else
    {
        eStatus = MB_EIO;
    }

    //printf("eStatus in eMBRTUReceive is %d\n",eStatus);

    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 eMBRTUSend
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
eMBErrorCode eMBRTUSend( UCHAR ucSlaveAddress, const UCHAR * pucFrame, USHORT usLength )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          usCRC16;
    int i =0;

    ENTER_CRITICAL_SECTION( );
    printf("Inside eMBRTUSend\n");

    /*for(i=0;i<usLength;i++)
    {
    	printf("Sending data is %x\n",*pucFrame++);
    }*/

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
    if( eRcvState == STATE_RX_IDLE )
    {
        /* First byte before the Modbus-PDU is the slave address. */
        pucSndBufferCur = ( UCHAR * ) pucFrame - 1;
        usSndBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pucSndBufferCur[MB_SER_PDU_ADDR_OFF] = ucSlaveAddress;
        usSndBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        usCRC16 = usMBCRC16( ( UCHAR * ) pucSndBufferCur, usSndBufferCount );
        ucRTUBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
        ucRTUBuf[usSndBufferCount++] = ( UCHAR )( usCRC16 >> 8 );

        /*for(i=0;i<8;i++)
        {
        	printf("Sending data %d\n",ucRTUBuf[i]);
        }*/

        printf("Buffer count is %d\n",usSndBufferCount);
        /* Activate the transmitter. */
        eSndState = STATE_TX_XMIT;
        vMBPortSerialEnable( FALSE, TRUE );
    }
    else
    {
        eStatus = MB_EIO;
    }
    EXIT_CRITICAL_SECTION(  );
    return eStatus;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 xMBRTUReceiveFSM
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
BOOL xMBRTUReceiveFSM( void )
{
    BOOL            xTaskNeedSwitch = FALSE;
    UCHAR           ucByte;

    assert( eSndState == STATE_TX_IDLE );

    /* Always read the character. */
    (void)xMBPortSerialGetByte( ( CHAR * ) & ucByte );
    //printf("Received byte is %x\n",ucByte);
    switch (eRcvState)
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
		case STATE_RX_INIT:
			vMBPortTimersEnable();
			break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
		case STATE_RX_ERROR:
			vMBPortTimersEnable();
			break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVE.
         */
		case STATE_RX_IDLE:
			ucRTUBuf[usRcvBufferPos++] = ucByte;
			eRcvState = STATE_RX_RCV;

			/* Enable t3.5 timers. */
			vMBPortTimersEnable();
			break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV:
        if( usRcvBufferPos < MB_SER_PDU_SIZE_MAX )
        {
            ucRTUBuf[usRcvBufferPos++] = ucByte;
        }
        else
        {
            eRcvState = STATE_RX_ERROR;
        }
        vMBPortTimersEnable();
        break;
    }


    return xTaskNeedSwitch;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 xMBRTUTransmitFSM
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
BOOL xMBRTUTransmitFSM( void )
{
    BOOL            xNeedPoll = FALSE;

    assert( eRcvState == STATE_RX_IDLE );
    switch ( eSndState )
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        vMBPortSerialEnable( TRUE, FALSE );
        break;

    case STATE_TX_XMIT:
        /* check if we are finished. */
        if( usSndBufferCount != 0 )
        {
            xMBPortSerialPutByte( ( CHAR )*pucSndBufferCur );
            pucSndBufferCur++;  /* next byte in sendbuffer. */
            usSndBufferCount--;
            //printf("Buffer count is %d\n",usSndBufferCount);
        }
        else
        {
            xNeedPoll = xMBPortEventPost( EV_FRAME_SENT );
            /* Disable transmitter. This prevents another transmit buffer
             * empty interrupt. */
            vMBPortSerialEnable( TRUE, FALSE );
            eSndState = STATE_TX_IDLE;
        }
        break;
    }

    return xNeedPoll;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 xMBRTUTimerT35Expired
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
BOOL xMBRTUTimerT35Expired( void )
{
    BOOL    xNeedPoll = FALSE;
    printf("eRcvState is %d\n",eRcvState);
    switch ( eRcvState )
    {
        /* Timer t35 expired. Startup phase is finished. */
    case STATE_RX_INIT:
        xNeedPoll = xMBPortEventPost( EV_READY );
        break;

        /* A frame was received and t35 expired. Notify the listener that
         * a new frame was received. */
    case STATE_RX_RCV:
        xNeedPoll = xMBPortEventPost( EV_FRAME_RECEIVED );
        break;

        /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:
        break;

        /* Function called in an illegal state. */
    default:
        assert( ( eRcvState == STATE_RX_INIT ) ||
                ( eRcvState == STATE_RX_RCV ) || ( eRcvState == STATE_RX_ERROR ) );
    }

    vMBPortTimersDisable(  );
    eRcvState = STATE_RX_IDLE;

    return xNeedPoll;
}

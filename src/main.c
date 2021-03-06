/**
 * @file 				main.c
 * FreeModbus Libary: BARE Demo Application
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: demo.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- System includes --------------------------------*/

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define REG_INPUT_START 1000
#define REG_INPUT_NREGS 4
#define SLAVE_ID 		0x0A

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usRegInputStart = REG_INPUT_START;
static USHORT   usRegInputBuf[REG_INPUT_NREGS];



/* ----------------------- Start implementation -----------------------------*/
/**
 -----------------------------------------------------------------------------------------------------------------------
 main
 -----------------------------------------------------------------------------------------------------------------------
*   Main entry
*
* 	@date       			DEC/02/2013
* 	@author                 ankit
* 	@pre                    None
* 	@return	 				None
************************************************************************************************************************
*/
int main( void )
{
    eMBErrorCode    eStatus;

    eStatus = eMBInit( MB_RTU, SLAVE_ID, 0, 9600, MB_PAR_NONE );

    /* Enable the Modbus Protocol Stack. */
    eStatus = eMBEnable();
    
    // Initialise some registers
    usRegInputBuf[1] = 0x1234;
    usRegInputBuf[2] = 0x5678;
    usRegInputBuf[3] = 0x9abc;        

    for( ;; )
    {
        (void)eMBPoll();
                
        /* Here we simply count the number of poll cycles. */
        usRegInputBuf[0]++;
    }
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 eMBRegInputCB
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START ) && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 eMBRegHoldingCB
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if (eMode == MB_REG_READ)
    {
        if( ( usAddress >= REG_INPUT_START )&& ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
        {
            iRegIndex = ( int )( usAddress - usRegInputStart );
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
        }
    }

    if (eMode == MB_REG_WRITE)
    {
        if( ( usAddress >= REG_INPUT_START )&& ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
        {
            iRegIndex = ( int )( usAddress - usRegInputStart );
            while( usNRegs > 0 )
            {
                usRegInputBuf[iRegIndex] =  ((unsigned int) *pucRegBuffer << 8) | ((unsigned int) *(pucRegBuffer+1));
                pucRegBuffer+=2;
                iRegIndex++;
                usNRegs--;
            }
        }
    }

    return eStatus;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 eMBRegCoilsCB
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,eMBRegisterMode eMode )
{
    printf("Inside eMBRegCoilsCB\n ");
    *pucRegBuffer = 0xAF;
	//return MB_ENOREG;
    return MB_ENOERR;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 eMBRegDiscreteCB
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    return MB_ENOREG;
}


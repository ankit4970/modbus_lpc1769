/**
 * @file 				portserial.c
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
#include "LPC17xx.h"
#include "portserial.h"
#include "mbutils.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );
static void prvvUARTISR( void );

/* ----------------------- System Variables ---------------------------------*/
//Serial pc(USBTX, USBRX);            	// Cam - mbed USB serial port

//Ticker simISR;                      	// Cam - mbed ticker
                                    	// we don't have the TX buff empty interrupt, so
                                    	// we just interrupt every 1 mSec and read RX & TX
                                    	// status to simulate the proper ISRs.

static BOOL RxEnable, TxEnable;     	// Cam - keep a static copy of the RxEnable and TxEnable
                                    	// status for the simulated ISR (ticker)


#define INTERNAL_CLOCK		    (4  * 1000 * 1000UL)    ///< Do not change, this is the same on all LPC17XX
#define EXTERNAL_CLOCK          (12 * 1000 * 1000UL)    ///< Change according to your board specification
#define RTC_CLOCK               (32768UL)               ///< Do not change, this is the typical RTC crystal value

#define SYSCTL_PLL0STS_ENABLED   (1 << 24)	/*!< PLL0 enable flag */
#define SYSCTL_PLL0STS_CONNECTED (1 << 25)	/*!< PLL0 connect flag */
#define SYSCTL_PLL0STS_LOCKED    (1 << 26)	/*!< PLL0 connect flag */
#define SYSCTL_PLL1STS_ENABLED   (1 << 8)	/*!< PLL1 enable flag */
#define SYSCTL_PLL1STS_CONNECTED (1 << 9)	/*!< PLL1 connect flag */
#define SYSCTL_PLL1STS_LOCKED    (1 << 10)	/*!< PLL1 connect flag */

/**
 * Peripheral clock selection for LPC175x/6x
 * This is a list of clocks that can be divided on the 175x/6x
 */
typedef enum {
	SYSCTL_PCLK_WDT,		/*!< Watchdog divider */
	SYSCTL_PCLK_TIMER0,	/*!< Timer 0 divider */
	SYSCTL_PCLK_TIMER1,	/*!< Timer 1 divider */
	SYSCTL_PCLK_UART0,	/*!< UART 0 divider */
	SYSCTL_PCLK_UART1,	/*!< UART 1 divider */
	SYSCTL_PCLK_RSVD5,
	SYSCTL_PCLK_PWM1,		/*!< PWM 1 divider */
	SYSCTL_PCLK_I2C0,		/*!< I2C 0 divider */
	SYSCTL_PCLK_SPI,		/*!< SPI divider */
	SYSCTL_PCLK_RSVD9,
	SYSCTL_PCLK_SSP1,		/*!< SSP 1 divider */
	SYSCTL_PCLK_DAC,		/*!< DAC divider */
	SYSCTL_PCLK_ADC,		/*!< ADC divider */
	SYSCTL_PCLK_CAN1,		/*!< CAN 1 divider */
	SYSCTL_PCLK_CAN2,		/*!< CAN 2 divider */
	SYSCTL_PCLK_ACF,		/*!< ACF divider */
	SYSCTL_PCLK_QEI,		/*!< QEI divider */
	SYSCTL_PCLK_GPIOINT,	/*!< GPIOINT divider */
	SYSCTL_PCLK_PCB,		/*!< PCB divider */
	SYSCTL_PCLK_I2C1,		/*!< I2C 1 divider */
	SYSCTL_PCLK_RSVD20,
	SYSCTL_PCLK_SSP0,		/*!< SSP 0 divider */
	SYSCTL_PCLK_TIMER2,	/*!< Timer 2 divider */
	SYSCTL_PCLK_TIMER3,	/*!< Timer 3 divider */
	SYSCTL_PCLK_UART2,	/*!< UART 2 divider */
	SYSCTL_PCLK_UART3,	/*!< UART 3 divider */
	SYSCTL_PCLK_I2C2,		/*!< I2C 2 divider */
	SYSCTL_PCLK_I2S,		/*!< I2S divider */
	SYSCTL_PCLK_RSVD28,
	SYSCTL_PCLK_RIT,		/*!< Repetitive timer divider */
	SYSCTL_PCLK_SYSCON,	/*!< SYSCON divider */
	SYSCTL_PCLK_MCPWM		/*!< Motor control PWM divider */
} CHIP_SYSCTL_PCLK_T;

/* UART Parity enable bit definitions */
#define UART_LCR_PARITY_EN      (1 << 3)		/*!< UART Parity Enable */
#define UART_LCR_PARITY_DIS     (0 << 3)		/*!< UART Parity Disable */
#define UART_LCR_PARITY_ODD     (0 << 4)		/*!< UART Parity select: Odd parity */
#define UART_LCR_PARITY_EVEN    (1 << 4)		/*!< UART Parity select: Even parity */
#define UART_LCR_PARITY_F_1     (2 << 4)		/*!< UART Parity select: Forced 1 stick parity */
#define UART_LCR_PARITY_F_0     (3 << 4)		/*!< UART Parity select: Forced 0 stick parity */
#define UART_LCR_BREAK_EN       (1 << 6)		/*!< UART Break transmission enable */
#define UART_LCR_DLAB_EN        (1 << 7)		/*!< UART Divisor Latches Access bit enable */
#define UART_LCR_BITMASK        (0xFF)			/*!< UART line control bit mask */

typedef enum CHIP_SYSCTL_CCLKSRC {
	SYSCTL_CCLKSRC_SYSCLK,		/*!< Select Sysclk as the input to the CPU clock divider. */
	SYSCTL_CCLKSRC_MAINPLL,		/*!< Select the output of the Main PLL as the input to the CPU clock divider. */
} CHIP_SYSCTL_CCLKSRC_T;

/**
 * Clock and power peripheral clock divider rates used with the
 * Clock_CLKDIVSEL_T clock types (devices only)
 */
typedef enum {
	SYSCTL_CLKDIV_4,			/*!< Divider by 4 */
	SYSCTL_CLKDIV_1,			/*!< Divider by 1 */
	SYSCTL_CLKDIV_2,			/*!< Divider by 2 */
	SYSCTL_CLKDIV_8,			/*!< Divider by 8, not for use with CAN */
	SYSCTL_CLKDIV_6_CCAN = SYSCTL_CLKDIV_8	/*!< Divider by 6, CAN only */
} CHIP_SYSCTL_CLKDIV_T;

/* ----------------------- System Variables ---------------------------------*/
//Timeout toMBUS;             	// Cam - mbed timeout
static ULONG usInterval;    	// Cam - timeout interval in microseconds

const uint32_t OscRateIn = 12000000;

/**
 * PLL source clocks
 */
typedef enum CHIP_SYSCTL_PLLCLKSRC {
	SYSCTL_PLLCLKSRC_IRC,			/*!< PLL is sourced from the internal oscillator (IRC) */
	SYSCTL_PLLCLKSRC_MAINOSC,		/*!< PLL is sourced from the main oscillator */
	SYSCTL_PLLCLKSRC_RTC,			/*!< PLL is sourced from the RTC oscillator */
	SYSCTL_PLLCLKSRC_RESERVED1,
	SYSCTL_PLLCLKSRC_RESERVED2
} CHIP_SYSCTL_PLLCLKSRC_T;



static inline BOOL Chip_Clock_IsMainPLLEnabled(void)
{
	return (BOOL) ((LPC_SC->PLL0STAT & SYSCTL_PLL0STS_ENABLED) != 0);
}

/**
 * @brief	Returns the main oscillator clock rate
 * @return	main oscillator clock rate
 */
static inline uint32_t Chip_Clock_GetMainOscRate(void)
{
	return OscRateIn;
}

/**
 * @brief	Returns the input clock source for SYSCLK
 * @return	input clock source for SYSCLK
 */
static inline CHIP_SYSCTL_PLLCLKSRC_T Chip_Clock_GetMainPLLSource(void)
{
	return (CHIP_SYSCTL_PLLCLKSRC_T) (LPC_SC->CLKSRCSEL & 0x3);
}

/* Returns the current SYSCLK clock rate */
uint32_t Chip_Clock_GetSYSCLKRate(void)
{

	return Chip_Clock_GetMainOscRate();

}
/**
 * @brief	Return Main PLL (PLL0) input clock rate
 * @return	PLL0 input clock rate
 */
static inline uint32_t Chip_Clock_GetMainPLLInClockRate(void)
{
	return Chip_Clock_GetSYSCLKRate();
}

/* Returns the main PLL output clock rate */
uint32_t Chip_Clock_GetMainPLLOutClockRate(void)
{
	uint32_t clkhr = 0;

	/* Only valid if enabled */
	if (Chip_Clock_IsMainPLLEnabled())
	{
		uint32_t msel, nsel;

		/* PLL0 rate is (FIN * 2 * MSEL) / NSEL, get MSEL and NSEL */
		msel = 1 + (LPC_SC->PLL0CFG & 0x7FFF);
		nsel = 1 + ((LPC_SC->PLL0CFG >> 16) & 0xFF);
		clkhr = (Chip_Clock_GetMainPLLInClockRate() * 2 * msel) / nsel;
	}
	return (uint32_t) clkhr;
}

/**
 * @brief	Read PLL0 connect status
 * @return	true of the PLL0 is connected. false if not connected
 */
static inline BOOL Chip_Clock_IsMainPLLConnected()
{
	return (BOOL) ((LPC_SC->PLL0STAT & SYSCTL_PLL0STS_CONNECTED) != 0);
}

/* Returns the current CPU clock source */
CHIP_SYSCTL_CCLKSRC_T Chip_Clock_GetCPUClockSource(void)
{
	CHIP_SYSCTL_CCLKSRC_T src;

	/* LPC175x/6x CPU clock source is based on PLL connect status */
	if (Chip_Clock_IsMainPLLConnected())
	{
		src = SYSCTL_CCLKSRC_MAINPLL;
	}
	else {
		src = SYSCTL_CCLKSRC_SYSCLK;
	}

	return src;
}

uint32_t Chip_Clock_GetMainClockRate(void)
{
	switch (Chip_Clock_GetCPUClockSource()) {
	case SYSCTL_CCLKSRC_MAINPLL:
		return Chip_Clock_GetMainPLLOutClockRate();

	case SYSCTL_CCLKSRC_SYSCLK:
		return Chip_Clock_GetSYSCLKRate();

	default:
		return 0;
	}
}

/* Gets the CPU clock divider */
uint32_t Chip_Clock_GetCPUClockDiv(void)
{

	return (LPC_SC->CCLKCFG & 0xFF) + 1;

}

/* Get CCLK rate */
uint32_t Chip_Clock_GetSystemClockRate(void)
{
	printf("Main clock rate is %d\n",Chip_Clock_GetMainClockRate());
	printf("Chip_Clock_GetCPUClockDiv is %d\n",Chip_Clock_GetCPUClockDiv());
	return Chip_Clock_GetMainClockRate() / Chip_Clock_GetCPUClockDiv();
}

/* Gets a clock divider for a peripheral */
uint32_t Chip_Clock_GetPCLKDiv(CHIP_SYSCTL_PCLK_T clk)
{
	uint32_t div = 1, bitIndex, regIndex = ((uint32_t) clk) * 2;

	/* Get register array index and clock index into the register */
	bitIndex = regIndex % 32;
	regIndex = regIndex / 32;

	/* Mask and update register */
	div = LPC_SC->PCLKSEL1;
	div = (div >> bitIndex) & 0x3;
	if (div == SYSCTL_CLKDIV_4)
	{
		div = 4;
	}
	else if (div == SYSCTL_CLKDIV_1)
	{
		div = 1;
	}
	else if (div == SYSCTL_CLKDIV_2)
	{
		div = 2;
	}
	else
	{
		/* Special case for CAN clock divider */
		if ((clk == SYSCTL_PCLK_CAN1) || (clk == SYSCTL_PCLK_CAN2) || (clk == SYSCTL_PCLK_ACF))
		{
			div = 6;
		}
		else
		{
			div = 8;
		}
	}

	return div;
}

/* Returns clock ID for the peripheral block */
static CHIP_SYSCTL_PCLK_T Chip_UART_GetClkIndex(LPC_UART_TypeDef *pUART)
{
	CHIP_SYSCTL_PCLK_T clkUART;

	if (pUART == LPC_UART1)
	{
		clkUART = SYSCTL_PCLK_UART1;
	}
	else if (pUART == LPC_UART2)
	{
		clkUART = SYSCTL_PCLK_UART2;
	}
	else if (pUART == LPC_UART3)
	{
		clkUART = SYSCTL_PCLK_UART3;
	}
	else
	{
		clkUART = SYSCTL_PCLK_UART0;
	}

	return clkUART;
}

/* Returns the clock rate for a peripheral */
uint32_t Chip_Clock_GetPeripheralClockRate(CHIP_SYSCTL_PCLK_T clk)
{
	/* 175x/6x clock is derived from CPU clock with CPU divider */
	printf("Chip_Clock_GetSystemClockRate is %d\n",Chip_Clock_GetSystemClockRate());
	printf("Chip_Clock_GetPCLKDiv(clk) is %d\n",Chip_Clock_GetPCLKDiv(clk));
	return Chip_Clock_GetSystemClockRate() / Chip_Clock_GetPCLKDiv(clk);
}

/* Determines and sets best dividers to get a target bit rate */
uint32_t Chip_UART_SetBaud(LPC_UART_TypeDef *pUART, uint32_t baudrate)
{
	uint32_t div, divh, divl, clkin;

	/* Determine UART clock in rate without FDR */

	clkin = Chip_Clock_GetPeripheralClockRate(Chip_UART_GetClkIndex(pUART));

	printf("Clock rate is %d\n",clkin);
	div = clkin / (baudrate * 16);

	/* High and low halves of the divider */
	divh = div / 256;
	divl = div - (divh * 256);

	//Chip_UART_EnableDivisorAccess(pUART);
	LPC_UART2->LCR |= UART_LCR_DLAB_EN;
	//Chip_UART_SetDivisorLatches(pUART, divl, divh);
	LPC_UART2->DLL = (uint32_t) divl;
	LPC_UART2->DLM = (uint32_t) divh;
	//Chip_UART_DisableDivisorAccess(pUART);
	LPC_UART2->LCR &= ~UART_LCR_DLAB_EN;

	/* Fractional FDR already setup for 1 in UART init */

	return clkin / div;
}

/* ----------------------- Start implementation -----------------------------*/
// Cam - This is called every 1mS to simulate Rx character received ISR and
// Tx buffer empty ISR.
/**
 -----------------------------------------------------------------------------------------------------------------------
 prvvUARTISR
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
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

/**
 -----------------------------------------------------------------------------------------------------------------------
 vMBPortSerialEnable
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
void vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
    //RxEnable = xRxEnable;
    //TxEnable = xTxEnable;

    if( xRxEnable )
	{
    	LPC_UART2->IER |= 0x01;
	}
	else
	{
		LPC_UART2->IER &= ~0x01;
	}
	if( xTxEnable )
	{
		LPC_UART2->IER |= 0x02;
		prvvUARTTxReadyISR(  );
	}
	else
	{
		LPC_UART2->IER &= ~0x02;
	}
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 xMBPortSerialInit
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
BOOL xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	unsigned int baud = 0;
	const unsigned int pclk = sys_get_cpu_clock();
    //simISR.attach_us(&prvvUARTISR,1000);    // Cam - attach prvvUARTISR to a 1mS ticker to simulate serial interrupt behaviour
                                            // 1mS is just short of a character time at 9600 bps, so quick enough to pick
                                            // up status on a character by character basis.
	printf("Initializing serial port\n");
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
	//LPC_UART2->LCR &=~ (3<<0 | 1<<7);
	LPC_UART2->LCR = 0x03;
	// LCR bit o and 1, Word length select
	//LPC_UART2->LCR |= (3<<0);						//00 5-bit character length
													//01 6-bit character length
													//10 7-bit character length
													//11 8-bit character length
	Chip_UART_SetBaud(LPC_UART2,ulBaudRate);
#if 0
	//Enable, Divisor latch access bit(DLAB)
	LPC_UART2->LCR |= (1<<7);
	baud = (pclk / (16 * ulBaudRate));

	//LPC_UART2->DLL = (baud & 0xFF);
	//LPC_UART2->DLM = (baud >> 8);

	LPC_UART2->DLL = 250;
	LPC_UART2->DLM = 2;

	LPC_UART2->LCR &=  ~(1<<7);
#endif
	LPC_UART2->FCR = 0x07;
	LPC_UART2->FCR &= ~(3<<6);
	//LPC_UART2->DLM = 0x00;
	//LPC_UART3->DLL =0x38;
	//LPC_UART2->DLL =( sys_get_cpu_clock() / (16 * ulBaudRate));

	// Step 4: FIFO not use:
	LPC_PINCON->PINSEL0 &=  ~(3<<20 | 3<<22);
	LPC_PINCON->PINSEL0 |= (1<<20 | 1<<22);    //TXD3 | RXD3
	LPC_PINCON->PINMODE4 &=  ~(3<<20 | 3<<22);
	LPC_PINCON->PINMODE4 |= (3<<22);           // 11 pull down

	// Step 6: Disable, Divisor latch access bit(DLAB)

	NVIC_EnableIRQ(UART2_IRQn);

	//enable receive data available interrupt for uart2
	LPC_UART2->IER &=~ (3<<0) ;
	LPC_UART2->IER |= (3<<0);

	//LPC_UART2->IER = (1 << 0) | (1 << 2); // B0:Rx, B1: Tx

    return TRUE;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 xMBPortSerialPutByte
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
BOOL xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    //pc.putc( ucByte);

	LPC_UART2->THR = ucByte;
	while( !(LPC_UART2->LSR & (1 << 5)));
	printf("Put byte %x\n",ucByte);
    return TRUE;
}

/**
 -----------------------------------------------------------------------------------------------------------------------
 xMBPortSerialGetByte
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
BOOL xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	//* pucByte = pc.getc();
	//while( !( LPC_UART2->LSR & 0x01 ) )
	//{
	//}

	*pucByte = LPC_UART2->RBR;
	printf("Received data is %d\n",*pucByte);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
/**
 -----------------------------------------------------------------------------------------------------------------------
 UART2_IRQHandler
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
void UART2_IRQHandler()
{
	uint32_t intsrc=0, tmp=0, tmp1=0;
	char           ucByte;
	volatile char   dummy;

	intsrc = (LPC_UART2->IIR & 0x03CF);
	tmp = intsrc & UART_IIR_INTID_MASK;
//	printf("Uart interrupt\n");
//	printf("Interrupt source intsrc is %x\n",intsrc);
//	printf("Interrupt source tmp is %x\n",tmp);
	 /* Always read the character. */
	//(void)xMBPortSerialGetByte( ( CHAR * ) & ucByte );
	//printf("Received byte is %c\n",ucByte);

	// Receive Line Status
	if (tmp == UART_IIR_INTID_RLS)
	{
		// Check line status
		tmp1 = ((LPC_UART2->LSR) & UART_LSR_BITMASK);
		// Mask out the Receive Ready and Transmit Holding empty status
		tmp1 &= (UART_LSR_OE | UART_LSR_PE | UART_LSR_FE | UART_LSR_BI | UART_LSR_RXFE);
		// If any error exist
		if (tmp1)
		{
			while(1);
		}
	}

	// Receive Data Available or Character time-out
	if ((tmp == UART_IIR_INTID_RDA) || (tmp == UART_IIR_INTID_CTI))
	{
		prvvUARTRxISR( );
		//xMBPortSerialGetByte( ( CHAR * ) & ucByte );
		//printf("Received byte is %c\n",ucByte);

	}

	// Transmit Holding Empty
	if (tmp == UART_IIR_INTID_THRE)
	{
		prvvUARTTxReadyISR();
	}


}

/**
 -----------------------------------------------------------------------------------------------------------------------
 prvvUARTTxReadyISR
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
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
/**
 -----------------------------------------------------------------------------------------------------------------------
 prvvUARTRxISR
 -----------------------------------------------------------------------------------------------------------------------
*   Event Handler for GPI module
*
* 	@date       			DEC/02/2013
* 	@author                         FW_DEV_2
* 	@pre                            None
* 	@return	 			None
************************************************************************************************************************
*/
static void prvvUARTRxISR(void)
{
    pxMBFrameCBByteReceived( );
}



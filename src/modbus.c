/*
===============================================================================
 Name        : modbus.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>

#include <stdio.h>

// TODO: insert other include files here

// TODO: insert other definitions and declarations here
#if 0
int main(void)
{

    printf("Hello World\n");

    // Force the counter to be placed into memory
    volatile static int i = 0 ;
    // Enter an infinite loop, just incrementing a counter
    while(1) {
        i++ ;
    }
    return 0 ;
}
#endif
/*
===================================================================================
 Name        : Receive
 Author      : Bharat Khanna
 Version     :
 Copyright   : $(copyright)
 Description : Receiving data, descrambe the data, and implement the LISA Algorithm
               display the message received
====================================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif
#include <cr_section_macros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "extint.h"
#include "timer.h"
#include<stdio.h>
#include "LPC17xx.h"
#include<string.h>
//#include"core_cm3.h"
char ReadChar;
#include<stdbool.h>
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


void UART2_IRQHandler()
{
	//char ReadChar;
	// Receiver data read bit number 0
	// Is set when UnRBR holds an unread character
	ReadChar = LPC_UART2->RBR;
	printf("%c", ReadChar);
}

bool init()
{
	const uint32_t  Baud_Rate = 9600;
	// Step 1:
	//UART2 power/clock control bit(24) of PCONP
	LPC_SC->PCONP &=~ (1<<24);
	LPC_SC->PCONP |= (1<<24);
	// Step 2:
	// PCLKSEL1 bit 16 and 17 of Peripheral Clock Selection register1 for Uart2, controls the rate of the clock signal
	// that will be supplied to the corresponding peripheral
	//Function peripheral clock selection
	// 00 CCLK/4
	// 01 CCLK
	// 10 CCLK/4
	// 11 CLK/4
	LPC_SC->PCLKSEL1 &=~(3<<16);
	LPC_SC->PCLKSEL1 |= (1<<16);

	// Step 3:// table 280
	LPC_UART2->LCR &=~ (3<<0 | 1<<7);
	// LCR bit o and 1, Word length select
	//00 5-bit character length
	//01 6-bit character length
	//10 7-bit character length
	//11 8-bit character length
	LPC_UART2->LCR |= (3<<0);
	//Enable, Divisor latch access bit(DLAB)
	LPC_UART2->LCR |= (1<<7);
	LPC_UART2->DLM = 0x00;
	//LPC_UART3->DLL =0x38;
	LPC_UART2->DLL =( sys_get_cpu_clock() / (16 * Baud_Rate));
	// Step 4:
	// FIFO not used
	// Step 5:
	LPC_PINCON->PINSEL0 &=~ (3<<20 | 3<<22);
	LPC_PINCON->PINSEL0 |= (1<<20 | 1<<22);    //TXD3 | RXD3
	LPC_PINCON->PINMODE4 &=~ (3<<20 | 3<<22);
	LPC_PINCON->PINMODE4 |= (3<<22);           // 11 pull down
	// Step 6:
	//Disable, Divisor latch access bit(DLAB)
	LPC_UART2->LCR &=~ (1<<7);
	NVIC_EnableIRQ(UART2_IRQn);
	//enable receive data available interrupt for uart2
	LPC_UART2->IER &=~ (1<<0);
	LPC_UART2->IER |= (1<<0);

	return true;
}

//int uart2_putchar(char out)
void uart2_putchar(char out)
{
	// Transmitter Empty bit number 6
	// Is set when both UnTHR and UnTSR are empty
	//RBR DLL=0
	LPC_UART2->THR = out;
	while( !(LPC_UART2->LSR & (1 << 5)));
	//return 1;
}


bool run()
{
	init();
	int i;
	char myString[] = "Hey, Hope you are doing well\n";
	int length = strlen(myString);
	for( i = 0 ; i < length ; i++ )
	{
		char writechar = myString[i];
		//uart2_putchar(writechar);
		delay(100);
	}

	return true;
}
#if 0
int main(void)
{
	while(1)
	{
		run();
	}

   return -1;
}
#endif

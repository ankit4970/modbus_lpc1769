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


#if 0
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

int main(void)
{
	while(1)
	{
		run();
	}

   return -1;
}
#endif

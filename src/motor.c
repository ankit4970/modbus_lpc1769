/*
 * motor.c
 *
 *  Created on: Oct 15, 2016
 *      Author: ankit
 */



/*
===============================================================================
 Name        : pwm.c
 Author      : ankit
 Description : PWM code for LPC1769
===============================================================================
*/


#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>

#include <stdio.h>
#include <stdbool.h>
#include "motor.h"



/***************************************************************************************************
 *  Pin Assignments for Motor 0 (PWM1 channel 1)
 *
 *  P2.0 --> Motor 0 PWM
 *  P2.1 --> Motor 0 Enable
 *  P2.2 --> Motor 0 Break
 *  P2.3 --> Motor 0 Direction
 *  P2.4 <-- Motor 0 Hall Input
 *
 *  Pin Assignments for Motor 1 (PWM1 channel 6)
 *
 * 	P2.5 --> Motor 1 PWM
 *  P2.6 --> Motor 1 Enable
 *  P2.7 --> Motor 1 Break
 *  P2.8 --> Motor 1 Direction
 *  P2.10 <-- Motor 1 Hall Input
 *
 **************************************************************************************************
 * System clock is 100MHz
 * Peripheral clock is 100/8 = 12.5MHz
 *
 *	How to calculate value of MR0 for desired frequency
 * 	MR0 = Peripheral clock/desired frequency
 * 	   	= 12.5MHz/125KHz
 * 	   	= 100
 * 	MR0 = 100 --> frequency is 125KHz
 *
 * 	MR0 = Peripheral clock/desired frequency
 * 	   	= 12.5MHz/50KHz
 * 	   	= 250
 *
 * 	For Duty cycle change value in MRx register for channel x
 *************************************************************************************************/

/**************************************************************************************************
* @brief	wait for ms amount of milliseconds
* @param	ms : Time to wait in milliseconds
**************************************************************************************************/
static void delay_ms(unsigned int ms)
{
    unsigned int i,j;
    for(i=0;i<ms;i++)
        for(j=0;j<50000;j++);
}

/**************************************************************************************************
* @brief 	Initialize Motor0 PINS
**************************************************************************************************/
static void BLDMotCON0()
{
	// Set P2.1 as GPIO for Motor Enable
	LPC_PINCON->PINSEL4 &= ~(3<<2);
	// Set P2.1 as output
	LPC_GPIO2->FIODIR |= (1<<MOTOR0_ENABLE_BIT);
	LPC_GPIO2->FIOSET = (0x1<<MOTOR0_ENABLE_BIT);	// Intially motor disabled

	// Set P2.2 as GPIO for Break
	LPC_PINCON->PINSEL4 &= ~(3<<4);
	// Set P2.2 as output
	LPC_GPIO2->FIODIR |= (1<<MOTOR0_BREAK_BIT);
	// Set initial Value of P2.2 as 0 (No Brakes)
	LPC_GPIO2->FIOSET &= ~(0x1<<MOTOR0_BREAK_BIT);

	// Set P2.3 as GPIO for Motor Direction
	LPC_PINCON->PINSEL4 &= ~(3<<6);
	// Set P2.3 as output
	LPC_GPIO2->FIODIR |= (1<<MOTOR0_DIR_BIT);
	// Set initial Value of P2.3 as 1 (Forward Direction)
	LPC_GPIO2->FIOSET |= (0x1<<MOTOR0_DIR_BIT);

	// Set P2.4 as GPIO for Hall Input
	LPC_PINCON->PINSEL4 &= ~(3<<8);
	// Set P2.4 as input
	LPC_GPIO2->FIODIR &= ~(1<<MOTOR0_HALLIN_BIT);

	/************* PWM Initialization -- PWM1.1 on pin P2.0********/



	// PCLK_peripheral = CCLK/8,
	LPC_SC->PCLKSEL0 |= (3<<12);
	// Enable PCPWM1
	LPC_SC->PCONP |= (1<<6);
	// P2.0 -> PWM1.1
	LPC_PINCON->PINSEL4 &= ~(3<<0);
	LPC_PINCON->PINSEL4 |= (1<<0);

	// Enable Counter and PWM
	LPC_PWM1->TCR = (1<<1) ;

	// No Prescalar
	LPC_PWM1->PR  =  25-1;

	// Reset on PWMMR0, reset PWMTC if it matches MR0
	LPC_PWM1->MCR = (1<<1);

	// See calculation below
	LPC_PWM1->MR0 = 1000;
	// 50% duty cycle
	LPC_PWM1->MR1 = 500;

	LPC_PWM1->LER = ((1<<0) | (1<<1));
	LPC_PWM1->PCR = (1<<MOTOR0_PWM_ENABLE_BIT);
	LPC_PWM1->TCR = ((1<<0) | (1<<1) | (1<<3));
	LPC_PWM1->TCR = ((1<<0) | (1<<3));

}

/**************************************************************************************************
* @brief 	Initialize Motor1 PINS
**************************************************************************************************/
static void BLDMotCON1()
{
	// Set P2.6 as GPIO for Enable
	LPC_PINCON->PINSEL4 &= ~(3<<12);
	// Set P2.6 as output
	LPC_GPIO2->FIODIR |= (1<<MOTOR1_ENABLE_BIT);


	// Set P2.7 as GPIO for Break
	LPC_PINCON->PINSEL4 &= ~(3<<14);
	// Set P2.7 as output
	LPC_GPIO2->FIODIR |= (1<<MOTOR1_BREAK_BIT);
	// Set initial Value of P2.7 as 0 (No Brakes)
	LPC_GPIO2->FIOSET &= ~(0x1<<MOTOR1_BREAK_BIT);

	// Set P2.8 as GPIO for Direction
	LPC_PINCON->PINSEL4 &= ~(3<<16);
	// Set P2.8 as output
	LPC_GPIO2->FIODIR |= (1<<MOTOR1_DIR_BIT);
	// Set initial Value of P2.8 as 1 (Forward Direction)
	LPC_GPIO2->FIOSET |= (0x1<<MOTOR1_DIR_BIT);

	// Set P2.10 as GPIO for Hall Input
	LPC_PINCON->PINSEL4 &= ~(3<<20);
	// Set P2.10 as input
	LPC_GPIO2->FIODIR &= ~(1<<MOTOR1_HALLIN_BIT);

	/************* PWM Initialization -- PWM1.6 on pin P2.5********/

	// Enable PCPWM1
	LPC_SC->PCONP |= (1<<6);

	// PCLK_peripheral = CCLK/8,
	LPC_SC->PCLKSEL0 |= (3<<12);

	// P2.5 -> PWM1.6
	LPC_PINCON->PINSEL4 &= ~(3<<10);
	LPC_PINCON->PINSEL4 |= (1<<10);

	// Enable Counter and PWM
	//LPC_PWM1->TCR = 2;                    //counter reset

	// No Pre-scalar
	LPC_PWM1->PR  =  25-1;

	// Reset on PWMMR0, reset PWMTC if it matches MR0
	LPC_PWM1->MCR = (1<<1);

	// See calculation below
	LPC_PWM1->MR0 = 1000;
	// 50% duty cycle
	LPC_PWM1->MR6 = 100;

	LPC_PWM1->LER |= ((1<<0) | (1<<6));
	LPC_PWM1->PCR = (1<<MOTOR1_PWM_ENABLE_BIT);
	LPC_PWM1->TCR = ((1<<0) | (1<<3));

}
/**************************************************************************************************
* @brief		Read Hall sensor data from Motor 0
* @return 		True  : if value of hall pin is 1
* 				false : if value of hall pin is 0
**************************************************************************************************/
static bool MagSenDAT0()
{
	return (((LPC_GPIO2->FIOPIN) & (0x1<<MOTOR0_HALLIN_BIT)) ? (true) : (false)) ;
}

/**************************************************************************************************
* @brief		Read Hall sensor data from Motor 1
* @return 		True  : if value of hall pin is 1
* 				false : if value of hall pin is 0
**************************************************************************************************/
static bool MagSenDAT1()
{
	return (((LPC_GPIO2->FIOPIN) & (0x1<<MOTOR1_HALLIN_BIT)) ? (true) : (false)) ;
}

/**************************************************************************************************
* @brief	 	set duty cycle of PWM for Motor 0 according to given percentage value
* @param[in]	dutyCycle	Duty-cycle in % value(valid values 1 to 100)
* @return 		None
* @todo : change the name according to functionality
**************************************************************************************************/
static void BLDCMotDAT0(uint8_t dutyCycle)
{
	uint32_t temp=0, newValue=0;

	temp = LPC_PWM1->MR0;
	newValue = (uint32_t)(temp*dutyCycle)/100;
	printf("%d\n",newValue);
	LPC_PWM1->MR1 = newValue;
	LPC_PWM1->LER |= ((1<<0) | (1<<1));
	//LPC_PWM1->TCR |= ((1<<0) | (1<<3));
}

/**************************************************************************************************
* @brief	 	set duty cycle of PWM for Motor 1 according to given percentage value
* @param[in]	dutyCycle	Duty-cycle in % value(valid values 1 to 100)
* @return 		None
**************************************************************************************************/
static void BLDCMotDAT1(uint8_t dutyCycle)
{
	uint32_t temp=0, newValue=0;

	temp = LPC_PWM1->MR0;
	newValue = (uint32_t)(temp*dutyCycle)/100;
	printf("%d\n",newValue);
	LPC_PWM1->MR6 = newValue;
	LPC_PWM1->LER |= ((1<<0) | (1<<6));
	//LPC_PWM1->TCR |= ((1<<0) | (1<<3));
}
/**************************************************************************************************
* motorTask : Motor 0 Control task
**************************************************************************************************/
static void BLDCMotTask0( void *pvParameters )
{
	unsigned long ulReceivedValue;
	bool hallStatus = false;

	for( ;; )
	{
		delay_ms(5);					// @todo : remove this once sensors are available
		hallStatus = MagSenDAT0();
		printf("Hall sensor value is %d\n",(int)hallStatus);
	}
}

/**************************************************************************************************
* motorTask : Motor 1 Control task
**************************************************************************************************/
static void BLDCMotTask1( void *pvParameters )
{
	unsigned long ulReceivedValue;
	bool hallStatus = false;

	for( ;; )
	{
		delay_ms(5);
		hallStatus = MagSenDAT1();
		printf("Hall sensor value is %d\n",(int)hallStatus);
	}
}
/**************************************************************************************************
* main : Main program entry
**************************************************************************************************/
int motorInit(void)
{

	printf("System clock is %d\n",SystemCoreClock);

	// Initialize Motor 0, Motor1 PINs and PWM frequency
	BLDMotCON0();
	//BLDMotCON1();

	// Initially Motor Direction forward
	//MOTOR0_DIR_FORWARD();
	//MOTOR1_DIR_FORWARD();

	// Setting initial dutyCycle as 50% for both motors
	//BLDCMotDAT0(1);
	//BLDCMotDAT1(20);

	// Enable PWM output
	MOTOR0_PWM_ENABLE();
	//MOTOR1_PWM_ENABLE();

	/* Start the two tasks as described in the accompanying application note. */
	//xTaskCreate( BLDCMotTask0, ( signed char * ) "MotorTask0", configMINIMAL_STACK_SIZE, NULL, configMOTOR_TASK_PRIORITY, NULL );
	//xTaskCreate( BLDCMotTask1, ( signed char * ) "MotorTask1", configMINIMAL_STACK_SIZE, NULL, configMOTOR_TASK_PRIORITY, NULL );
	//MOTOR0_ENABLE();
	//MOTOR1_ENABLE();
	//delay_ms(1000);
	//BLDCMotDAT0(0);
	//delay_ms(3000);
	//BLDCMotDAT0(1);
	//MOTOR0_PWM_DISABLE();
	/* Start the tasks running. */
	//vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */

}


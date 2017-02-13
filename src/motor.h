/*
 * motor.h
 *
 *  Created on: Sep 29, 2016
 *      Author: ankit
 */

#ifndef MOTOR_H_
#define MOTOR_H_

/* Kernel includes. */
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
#include <stdio.h>
#include <stdbool.h>


/* Priorities at which the tasks are created. */
#define 	configQUEUE_RECEIVE_TASK_PRIORITY	( tskIDLE_PRIORITY + 2 )
#define		configQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define 	configMOTOR_TASK_PRIORITY			( tskIDLE_PRIORITY + 2 )
/* The rate at which data is sent to the queue, specified in milliseconds. */
#define mainQUEUE_SEND_FREQUENCY_MS				( 500 / portTICK_RATE_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH						( 1 )


#define SBIT_CNTEN     0
#define SBIT_PWMEN     2

#define SBIT_PWMMR0R   1

#define SBIT_LEN0      0
#define SBIT_LEN1      1
#define SBIT_LEN2      2
#define SBIT_LEN3      3
#define SBIT_LEN4      4

#define SBIT_PWMENA1   9
#define SBIT_PWMENA2   10
#define SBIT_PWMENA3   11
#define SBIT_PWMENA4   12

#define PWM_1          0 //P2_0 (0-1 Bits of PINSEL4)
#define PWM_2          2 //P2_1 (2-3 Bits of PINSEL4)
#define PWM_3          4 //P2_2 (4-5 Bits of PINSEL4)
#define PWM_4          6 //P2_3 (6-7 Bits of PINSEL4)

/********************************** Motor 0 Control Definitions ****************************************/

#define MOTOR0_ENABLE_BIT 		1
#define MOTOR0_BREAK_BIT 		2
#define MOTOR0_DIR_BIT 			3
#define MOTOR0_HALLIN_BIT 		4
#define MOTOR0_PWM_ENABLE_BIT	9	// Refer PWM1PCR register description for LPC176x/5x

#define MOTOR0_ENABLE()			(LPC_GPIO2->FIOCLR = (0x1<<MOTOR0_ENABLE_BIT))
#define MOTOR0_DISABLE()		(LPC_GPIO2->FIOSET = (0x1<<MOTOR0_ENABLE_BIT))

#define MOTOR0_BREAK_ENABLE()	(LPC_GPIO2->FIOSET |= (0x1<<MOTOR0_BREAK_BIT))
#define MOTOR0_BREAK_DISABLE()	(LPC_GPIO2->FIOCLR |= (0x1<<MOTOR0_BREAK_BIT))

#define MOTOR0_DIR_FORWARD()	(LPC_GPIO2->FIOSET = (0x1<<MOTOR0_DIR_BIT))
#define MOTOR0_DIR_REVERSE()	(LPC_GPIO2->FIOCLR = (0x1<<MOTOR0_DIR_BIT))

#define MOTOR0_PWM_ENABLE()		(LPC_PWM1->PCR |= ((1<<MOTOR0_PWM_ENABLE_BIT)))
#define MOTOR0_PWM_DISABLE()	(LPC_PWM1->PCR &= ~((1<<MOTOR0_PWM_ENABLE_BIT)))


/********************************** Motor 1 Control Definitions ****************************************/

#define MOTOR1_ENABLE_BIT 		6
#define MOTOR1_BREAK_BIT 		7
#define MOTOR1_DIR_BIT 			8
#define MOTOR1_HALLIN_BIT 		10
#define MOTOR1_PWM_ENABLE_BIT	14		// Refer PWM1PCR register description for LPC176x/5x

#define MOTOR1_ENABLE()			(LPC_GPIO2->FIOCLR |= (0x1<<MOTOR1_ENABLE_BIT))
#define MOTOR1_DISABLE()		(LPC_GPIO2->FIOSET |= (0x1<<MOTOR1_ENABLE_BIT))

#define MOTOR1_BREAK_ENABLE()	(LPC_GPIO2->FIOSET |= (0x1<<MOTOR1_BREAK_BIT))
#define MOTOR1_BREAK_DISABLE()	(LPC_GPIO2->FIOCLR |= (0x1<<MOTOR1_BREAK_BIT))

#define MOTOR1_DIR_FORWARD()	(LPC_GPIO2->FIOSET |= (0x1<<MOTOR1_DIR_BIT))
#define MOTOR1_DIR_REVERSE()	(LPC_GPIO2->FIOCLR |= (0x1<<MOTOR1_DIR_BIT))

#define MOTOR1_PWM_ENABLE()		(LPC_PWM1->PCR |= (1<<MOTOR1_PWM_ENABLE_BIT))
#define MOTOR1_PWM_DISABLE()	(LPC_PWM1->PCR &= ~(1<<MOTOR1_PWM_ENABLE_BIT))

typedef struct
{
	float SEN_Angular_angle;
	float SEN_direction_Angle;
} SEN_STRUCT ;


typedef struct
{
	bool	MOT_Rotation_direction;
	int  	MOT_RPM;
} MOT_STRUCT ;

typedef struct
{

} MOD_STRUCT;

int motorInit(void);
static void MagSenCON0( );
static void MagsenDAT0( );
static void AngSenCON0( );
static void AngSenDAT0( );
static void UltSenCON0( );
static void UltSenDAT0( );

static void BLDMotCON0( );
static void BLDMotDAT0( );
static void SerMotCON0( );
static void SerMotDAT0( );
static void StepMotCON0( );
static void StepMotDAT0( );

static void BLDMotCON1();
static void BLDMotDAT1( );
static void MagsenDAT1( );
#endif /* MOTOR_H_ */

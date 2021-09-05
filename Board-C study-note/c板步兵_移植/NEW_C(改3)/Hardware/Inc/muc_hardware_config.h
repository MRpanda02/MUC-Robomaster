#ifndef _MUC_HARDWARE_CONFIG_H__
#define _MUC_HARDWARE_CONFIG_H__


#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\


#define MyAbs(x)  ( (x)>0?(x):-(x) )

/*******************************************************************/
/*******************************************************************/
#if 1   /* Led Hardware Config */ 

#include "gpio.h"

#define LED_Red_On()        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET)
#define LED_Red_Off()       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET)
#define LED_Red_Toggle()    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define LED_Green_On()      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET)
#define LED_Green_Off()     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)
#define LED_Green_Toggle()  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)

#endif  /* Led Hardware Config END */ 


/*******************************************************************/
/*******************************************************************/
#if 1  /* Timer Config */ 

#include "tim.h"

#endif /* Timer Config END */ 

/*******************************************************************/
/*******************************************************************/
#if 1  /* Uart Config */ 

#include "usart.h"

#define DBUS_Uart huart1
#define Referee_Uart huart3
#define MiniPC_Uart huart6

#endif /* Uart Config END */ 

/*******************************************************************/
/*******************************************************************/
#if 1  /* Can Config */ 

#include "can.h"

#endif /* Can Config END */ 

/*******************************************************************/
/*******************************************************************/
#if 1  /* Debug Config */ 

#include "stdio.h" 

#endif /* Debug Config END */


#endif

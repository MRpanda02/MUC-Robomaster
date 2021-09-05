#ifndef _MUC_CONFIG_HARDWARE_H__
#define _MUC_CONFIG_HARDWARE_H__

/*******************************************************************/
/*******************************************************************/
#if 1   /* Led Hardware Config */ 

#include "gpio.h"

//#define LED_Red_On()        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET)
//#define LED_Red_Off()       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET)
//#define LED_Red_Toggle()    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
//#define LED_Green_On()      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET)
//#define LED_Green_Off()     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)
//#define LED_Green_Toggle()  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
//#define LED_Blue_On()      HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET)
//#define LED_Blue_Off()     HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET)
//#define LED_Blue_Toggle()  HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin)

#define LED_Red_Off()        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET)
#define LED_Red_On()       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET)
#define LED_Red_Toggle()    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define LED_Green_Off()      HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET)
#define LED_Green_On()     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)
#define LED_Green_Toggle()  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define LED_Blue_Off()      HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET)
#define LED_Blue_On()     HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET)
#define LED_Blue_Toggle()  HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin)

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


#endif //_MUC_CONFIG_GLOBAL_H__

#ifndef _MUC_HARDWARE_SUPER_CAPACITY_H__
#define _MUC_HARDWARE_SUPER_CAPACITY_H__
#include "main.h"

#define SuperCapInputOn()        HAL_GPIO_WritePin(SuperCap_Input_GPIO_Port, SuperCap_Input_Pin, GPIO_PIN_SET)
#define SuperCapInputOff()       HAL_GPIO_WritePin(SuperCap_Input_GPIO_Port, SuperCap_Input_Pin, GPIO_PIN_RESET)

#define SuperCapOutputOn()        HAL_GPIO_WritePin(SuperCap_Output_GPIO_Port, SuperCap_Output_Pin, GPIO_PIN_SET)
#define SuperCapOutputOff()       HAL_GPIO_WritePin(SuperCap_Output_GPIO_Port, SuperCap_Output_Pin, GPIO_PIN_RESET)


void SuperCapControlLoopTask( void );
void Deal_Data_ForShift( uint16_t ShiftFlag, uint8_t * msgData);
static  uint16_t ShiftToChassis=0; 

#endif  // _MUC_HARDWARE_SUPER_CAPACITY_H__

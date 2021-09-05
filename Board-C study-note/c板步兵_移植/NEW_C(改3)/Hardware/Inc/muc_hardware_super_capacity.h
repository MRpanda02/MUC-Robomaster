#ifndef _MUC_HARDWARE_SUPER_CAPACITY_H__
#define _MUC_HARDWARE_SUPER_CAPACITY_H__
#include "main.h"

#define SuperCapChargeUpOpen()        HAL_GPIO_WritePin(SuperCap_Charge_Up_GPIO_Port, SuperCap_Charge_Up_pin, GPIO_PIN_RESET)
#define SuperCapChargeUpClose()       HAL_GPIO_WritePin(SuperCap_Charge_Up_GPIO_Port, SuperCap_Charge_Up_pin, GPIO_PIN_SET)

#define SuperCapOutputOn()        HAL_GPIO_WritePin(SuperCap_Output_GPIO_Port, SuperCap_Output_Pin, GPIO_PIN_SET)
#define SuperCapOutputOff()       HAL_GPIO_WritePin(SuperCap_Output_GPIO_Port, SuperCap_Output_Pin, GPIO_PIN_RESET)


void SuperCapPWMControl( void );
void Deal_Data_ForShift( uint16_t ShiftFlag, uint8_t * msgData);
static  uint16_t ShiftToChassis=0; 

#endif  // _MUC_HARDWARE_SUPER_CAPACITY_H__

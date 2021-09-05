#ifndef _MUC_HARDWARE_CAN_H__
#define _MUC_HARDWARE_CAN_H__
#include "main.h"
#include "can.h"
void CanFilter_Init(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, uint16_t _id, int16_t message1,int16_t message2,int16_t message3,int16_t message4);

#endif  // _MUC_HARDWARE_CAN_H__

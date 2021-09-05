#ifndef _MUC_HARDWARE_UART_H__
#define _MUC_HARDWARE_UART_H__

#include "muc_config_hardware.h"

HAL_StatusTypeDef MucUartReceiveIT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void MucUsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size);

#endif  // _MUC_HARDWARE_UART_H__

#ifndef _MUC_HARDWARE_KEY_H__
#define _MUC_HARDWARE_KEY_H__
#include "main.h"

typedef enum
{
	BtnPressDown=0,
	BtnPressUp=1,
}ButtonSta_t;

#define KEY()	HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)

ButtonSta_t GetKeySta( void );
uint8_t GetKeyPressUpAction( void );

#endif  // _MUC_HARDWARE_KEY_H__

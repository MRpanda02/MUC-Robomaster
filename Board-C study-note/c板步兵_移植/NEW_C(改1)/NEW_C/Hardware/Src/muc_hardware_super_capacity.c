#include "muc_hardware_super_capacity.h"
#include "muc_remote.h"
#include "muc_hardware_can.h"
#include "tim.h"
#include "muc_chassis.h"
#include "muc_gimbal.h"

extern uint16_t ShiftToChassis; 

void Deal_Data_ForShift( uint16_t ShiftFlag, uint8_t * msgData)
{
ShiftFlag =(msgData[0]<<8)|msgData[1];
}

#define POWER_LIMIT         80.0f
#define WARNING_POWER       60.0f   
void SuperCapPWMControl( void )
{
	/*电容充电开放条件：输入口为低电平，地盘功率较小时

	电容放电开放条件：地盘功率超限，视超限程度利用PWM控制电容放电开关
	（高电平为电源放电，低电平为电容放电）
	占空比越低，电容放电越多
	中间有个最大值		重载值为1000*/
	int P1;
	float chassis_power = g_refereeSystemReceMesg.power_heat_data.chassis_power;
	float chassis_power_buffer = g_refereeSystemReceMesg.power_heat_data.chassis_power_buffer;
	//电容充电
	if(chassis_power < WARNING_POWER)
	{
		SuperCapChargeUpOpen();
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,1000);
	}
	else
	{
		SuperCapChargeUpClose();
	}
	if(chassis_power > WARNING_POWER && chassis_power < POWER_LIMIT)
	{
		//占空比比最大值大一点
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,800);
	}
	else
	{
		if(chassis_power_buffer > 40.0f)
		{
			//占空比接近最大值
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,700);
		}
		else
		{
			//占空比为中间最大值
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);
		}
	}
	
	P1 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5);
}

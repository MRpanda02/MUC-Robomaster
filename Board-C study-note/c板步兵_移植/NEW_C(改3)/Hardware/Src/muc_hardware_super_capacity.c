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
	/*���ݳ�翪�������������Ϊ�͵�ƽ�����̹��ʽ�Сʱ

	���ݷŵ翪�����������̹��ʳ��ޣ��ӳ��޳̶�����PWM���Ƶ��ݷŵ翪��
	���ߵ�ƽΪ��Դ�ŵ磬�͵�ƽΪ���ݷŵ磩
	ռ�ձ�Խ�ͣ����ݷŵ�Խ��
	�м��и����ֵ		����ֵΪ1000*/
	int P1;
	float chassis_power = g_refereeSystemReceMesg.power_heat_data.chassis_power;
	float chassis_power_buffer = g_refereeSystemReceMesg.power_heat_data.chassis_power_buffer;
	//���ݳ��
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
		//ռ�ձȱ����ֵ��һ��
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,800);
	}
	else
	{
		if(chassis_power_buffer > 40.0f)
		{
			//ռ�ձȽӽ����ֵ
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,700);
		}
		else
		{
			//ռ�ձ�Ϊ�м����ֵ
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,500);
		}
	}
	
	P1 = HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5);
}

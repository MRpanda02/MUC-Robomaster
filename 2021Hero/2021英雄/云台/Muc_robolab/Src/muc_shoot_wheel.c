#include "muc_shoot_wheel.h"
#include "muc_pid.h"
#include "muc_hardware_can.h"
#include "muc_remote.h"
#include "tim.h"
#include "muc_main_task.h"
#include "muc_stir_wheel.h"


#define LASER_ON()   HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET)
#define LASER_OFF()  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET)
//СĦ����
//#define SMA_LASER_ON()   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0, GPIO_PIN_SET)
//#define SMA_LASER_OFF()  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0, GPIO_PIN_RESET)

extern uint32_t mainTaskTimes;

//С�������
   //СĦ��������ֵ
#define SetFrictionWheelSpeed(x) \
        TIM1->CCR1 = x;                \
        TIM1->CCR2 = x;
				
#define FRICTION_WHEELRUNSPEED	1430// Ħ������ת�ٶ�����ֵ
#define FRICTION_WHEELSTOPVAL	  1000// Ħ����ֹͣ��ת����ֵ

WheelSta_t SmallWheelSta = 0;//Ħ����״̬
uint32_t SmallWheelRunTime = 0;//Ħ����ת��ʱ��

void SmallShootInit()
{
  //Ħ���ֳ�ʼ��
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

	SmallWheelSta = WheelStop;
	
	SetFrictionWheelSpeed( FRICTION_WHEELSTOPVAL );
  //���⴮�ڳ�ʼ��
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  GPIO_InitStruct.Pin = GPIO_PIN_0;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}




//�������
ShootSpeedSta_t shootStatus = Spd_Stop;//Ħ����״̬
WheelSta_t BigWheelSta = 0;//Ħ����״̬

ShootReceiveTypeDef Shoot_Left_Feedback={0};//����Ħ���ַ���ֵ
ShootReceiveTypeDef Shoot_Right_Feedback={0};

PidTypeDef Shoot_left_speed_pid={0};//����Ħ����pid��
PidTypeDef Shoot_right_speed_pid={0};

uint32_t BigWheelRunTime = 0;//Ħ����ת��ʱ��


void BigShootInit()//�������Ħ���ֳ�ʼ��
{
	PID_Init(&Shoot_left_speed_pid, 8 , 0.5f,  0,0,16000,0,0,16000);
	PID_Init(&Shoot_right_speed_pid,8 , 0.5f,  0,0,16000,0,0,16000);

//���������е㳬,������
//	PID_Init(&Shoot_left_speed_pid, 7.5 , 0.5f,  0,0,16000,0,0,16000);
//	PID_Init(&Shoot_right_speed_pid,7.5 , 0.5f,  0,0,16000,0,0,16000);

}

void Shoot_Speed_Data_deal(ShootReceiveTypeDef *Receive, uint8_t * msgData)//�������Ħ�������ݴ���
{
	Receive->real[Receive->count]=(msgData[2]<<8)|msgData[3];
	Receive->count++;
	if(	Receive->count>3)
	{
		Receive->calc=(Receive->real[0]+Receive->real[1]+Receive->real[2]+Receive->real[3])/4;
		Receive->count=0;
	}
	
}

ShootSpeedSta_t MucGetShootSpeed( void )//��ȡ����״̬
{
    return shootStatus;
}



//���������ʼ��
void MucShootInit( void )
{
	BigShootInit();
	SmallShootInit();
}

//��СĦ���ֿ��ƺ���:ͬʱ�����͹ر�
void Shoot_Loop_Task()
{
	static int CouldRun = 0;//�����Ħ���ֱ�־
	static int MouseRightPressTime = 0;
	
	static int BigLeftSpeed = 0;//��Ħ����Ŀ���ٶ�
	static int BigRightSpeed = 0;
	
	if(CC_Condition.S2Sta == 1)//ң����:
	{
		if(CC_Condition.S1Action == SwitchAction_3TO1)
		{
			//СĦ����
			if(SmallWheelSta == WheelStop)
			{ SmallWheelSta = WheelRun; }
			else
				{ SmallWheelSta = WheelStop; }
			
			//��Ħ����
			if(BigWheelSta == WheelStop)
			{ BigWheelSta = WheelRun; }
			else
				{ BigWheelSta = WheelStop; }
		}
	}
	else if(CC_Condition.S2Sta == 3)//����:�Ҽ���������,����SHIFT���Ҽ��ر�
		{
			if(SmallWheelSta == WheelStop && BigWheelSta == WheelStop && CouldRun == 1)//ֹͣ״̬��
			{
				if(CC_Condition.MouseRightBt == PressDown )
				{ 
					SmallWheelSta = WheelRun;
					BigWheelSta = WheelRun;
				}
			}
			else//ת��״̬��
			{
				if(CC_Condition.MouseRightBt == PressDown && CC_Condition.Key_SHIFT == PressDown)
				{ MouseRightPressTime++; }
				else
				{ MouseRightPressTime = 0; }
				
				if(MouseRightPressTime > 500)//SHIFT �� ����Ҽ������ر�
				{ 
					SmallWheelSta = WheelStop;
					BigWheelSta = WheelStop;
					CouldRun = 0;
					LASER_OFF();
				}
			}
			if(CouldRun == 0 && CC_Condition.MouseRightBt == PressUp)
			{ CouldRun = 1; }
		}
		else if(CC_Condition.S2Sta == 2)//ֹͣ
		{	
		  SmallWheelSta = WheelStop;
      BigWheelSta = WheelStop;			
		}
	
	//СĦ���ַ��͵���
	if(mainTaskTimes%10==0)
	{
		if(SmallWheelSta == WheelRun)
		{
			LASER_ON();
			SetFrictionWheelSpeed(FRICTION_WHEELRUNSPEED);  
		}
		else {
			LASER_OFF();
			SetFrictionWheelSpeed(FRICTION_WHEELSTOPVAL);
		}
	}	
	//��Ħ���ֵ�������
	if(BigWheelSta == WheelRun)
	{ 
		BigLeftSpeed = SHOOT_LEFT_CONFIG_FrictionWheelSpeed_HIGH;
    BigRightSpeed = SHOOT_RIGHT_CONFIG_FrictionWheelSpeed_HIGH;	
	}
	else
  { 
		BigLeftSpeed = 0;
    BigRightSpeed = 0;	
	}
	PID_Calc(&Shoot_left_speed_pid,Shoot_Left_Feedback.calc*2,BigLeftSpeed);//����Ħ���ֵ����PID����
  PID_Calc(&Shoot_right_speed_pid,Shoot_Right_Feedback.calc*2,BigRightSpeed);
    
	//ת��ʱ�����
	if(SmallWheelSta == WheelRun){//С
		SmallWheelRunTime ++;
	}
	else
	{
		SmallWheelRunTime = 0;
	}
	
	if(BigWheelSta == WheelRun){//��
		BigWheelRunTime ++;
	}
	else
	{
		BigWheelRunTime = 0;
	}

}
	
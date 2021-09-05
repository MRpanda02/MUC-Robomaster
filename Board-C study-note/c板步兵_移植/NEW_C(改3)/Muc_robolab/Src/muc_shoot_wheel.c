#include "muc_shoot_wheel.h"
#include "muc_pid.h"
#include "muc_hardware_can.h"
#include "muc_remote.h"
#include "tim.h"
#define LASER_ON()  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET)
#define LASER_OFF() HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET)

//×óÓÒÄ¦²ÁÂÖ·µ»ØÖµ
ShootReceiveTypeDef Shoot_Left_Feedback={0};
ShootReceiveTypeDef Shoot_Right_Feedback={0};

/*Ä¦²ÁÂÖ×´Ì¬*/
ShootSpeedSta_t shootStatus = Spd_Stop;

//×óÓÒÄ¦²ÁÂÖpid×é
PidTypeDef Shoot_left_speed_pid={0};
PidTypeDef Shoot_right_speed_pid={0};
int ShootWheelModeFlag=0;

void MucShootInit( void )
{
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1000);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1000);
}


void Shoot_Speed_Data_deal(ShootReceiveTypeDef *Receive, uint8_t * msgData)
{
	Receive->real[Receive->count]=(msgData[2]<<8)|msgData[3];
	Receive->count++;
	if(	Receive->count>3)
	{
		Receive->calc=(Receive->real[0]+Receive->real[1]+Receive->real[2]+Receive->real[3])/4;
		Receive->count=0;
	}
}



ShootSpeedSta_t MucGetShootSpeed( void )
{
    return shootStatus;
}

void MucSetShootSpeed( ShootSpeedSta_t sta )
{
    shootStatus = sta;
}


/*Ä¦²ÁÂÖÒ£¿ØÆ÷¿ØÖÆ£¬²»ÄÜµ÷½ÚËÙ¶È*/
void ShootAnalysisRemote(void)
{
	if(shootStatus==Spd_Stop)
	{			
		if(CC_Condition.S1Action == SwitchAction_3TO1)
		{
//			TIM4->CCR3 =1000;
			shootStatus = Spd_High;
			LASER_ON();
		}
	}else
	{
		if(CC_Condition.S1Action == SwitchAction_3TO1) 
		{
//			 TIM4->CCR3 =2000;
			shootStatus = Spd_Stop;
			LASER_OFF();
		}      
	}
		
}

void ShootAnalysisKey(void)
{
	static int MouseRightPressTime=0;	
	static int KeyF_PressTime=0;	
	static int Shoot_Speed_change=0;
	static int Shoot_Start_Station=0;
	static int first=0;

	if(	CC_Condition.Key_V  == PressDown&&Shoot_Start_Station==1)
	{
		MouseRightPressTime++;
		if(MouseRightPressTime>500)
		{
			shootStatus = Spd_Stop;
			Shoot_Start_Station=0;
			MouseRightPressTime=0;
			first=0;
			LASER_OFF();
		}
	}

	if(CC_Condition.Key_V == PressDown&&Shoot_Start_Station==0)
	{
		MouseRightPressTime++;
		if(MouseRightPressTime>1000)
		{
			
			Shoot_Start_Station=1;
			shootStatus =Spd_High ;
			MouseRightPressTime=0;
			first=1;
			LASER_ON();
	    }
	}


	
}
	  
void ShootAnalysisWay(void)
{
	if(g_RcRawData.s2==1)
	{
		ShootAnalysisRemote(); 
	}else if(g_RcRawData.s2==3)
	{
		ShootAnalysisKey();
	}
}

void Shoot_Loop_Task( void )
{
	static int RunningSlopeTime=0;
	static int RunningTime=0;
    static double RealShootWheelOut=0;    
	static double TargetShootWheelOut=0;

	RunningTime=(RunningTime+1)%1000;
	RunningSlopeTime=(RunningSlopeTime+1)%20;
	
	ShootAnalysisWay();
	
	
	
	
	if(MucGetShootSpeed()==Spd_High)
	{   ShootWheelModeFlag=0;
		 TargetShootWheelOut=1150;
		if(CC_Condition.Key_CTRL==PressDown)
		{
		ShootWheelModeFlag=1;
		 TargetShootWheelOut=1100;
		}

	}else
	{
    TargetShootWheelOut=1000;
	}
	
	if(RunningSlopeTime==0)
	{
	DataSlopeProcessing(RealShootWheelOut,TargetShootWheelOut,10);
	}
	
	if(RunningTime<500)
	{
	 __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,	(int)TargetShootWheelOut);
	}
	else
	{
     __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,	(int)TargetShootWheelOut);
	}
}



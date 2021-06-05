#include "muc_shoot_wheel.h"
#include "muc_pid.h"
#include "muc_hardware_can.h"
#include "muc_remote.h"
#include "tim.h"
#include "muc_main_task.h"
#include "muc_stir_wheel.h"


#define LASER_ON()   HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET)
#define LASER_OFF()  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET)
//小摩擦轮
//#define SMA_LASER_ON()   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0, GPIO_PIN_SET)
//#define SMA_LASER_OFF()  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0, GPIO_PIN_RESET)

extern uint32_t mainTaskTimes;

//小发射机构
   //小摩擦轮油门值
#define SetFrictionWheelSpeed(x) \
        TIM1->CCR1 = x;                \
        TIM1->CCR2 = x;
				
#define FRICTION_WHEELRUNSPEED	1430// 摩擦轮旋转速度设置值
#define FRICTION_WHEELSTOPVAL	  1000// 摩擦轮停止旋转设置值

WheelSta_t SmallWheelSta = 0;//摩擦轮状态
uint32_t SmallWheelRunTime = 0;//摩擦轮转动时间

void SmallShootInit()
{
  //摩擦轮初始化
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

	SmallWheelSta = WheelStop;
	
	SetFrictionWheelSpeed( FRICTION_WHEELSTOPVAL );
  //激光串口初始化
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  GPIO_InitStruct.Pin = GPIO_PIN_0;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}




//大发射机构
ShootSpeedSta_t shootStatus = Spd_Stop;//摩擦轮状态
WheelSta_t BigWheelSta = 0;//摩擦轮状态

ShootReceiveTypeDef Shoot_Left_Feedback={0};//左右摩擦轮返回值
ShootReceiveTypeDef Shoot_Right_Feedback={0};

PidTypeDef Shoot_left_speed_pid={0};//左右摩擦轮pid组
PidTypeDef Shoot_right_speed_pid={0};

uint32_t BigWheelRunTime = 0;//摩擦轮转动时间


void BigShootInit()//大发射机构摩擦轮初始化
{
	PID_Init(&Shoot_left_speed_pid, 8 , 0.5f,  0,0,16000,0,0,16000);
	PID_Init(&Shoot_right_speed_pid,8 , 0.5f,  0,0,16000,0,0,16000);

//连发射速有点超,调试用
//	PID_Init(&Shoot_left_speed_pid, 7.5 , 0.5f,  0,0,16000,0,0,16000);
//	PID_Init(&Shoot_right_speed_pid,7.5 , 0.5f,  0,0,16000,0,0,16000);

}

void Shoot_Speed_Data_deal(ShootReceiveTypeDef *Receive, uint8_t * msgData)//大发射机构摩擦轮数据处理
{
	Receive->real[Receive->count]=(msgData[2]<<8)|msgData[3];
	Receive->count++;
	if(	Receive->count>3)
	{
		Receive->calc=(Receive->real[0]+Receive->real[1]+Receive->real[2]+Receive->real[3])/4;
		Receive->count=0;
	}
	
}

ShootSpeedSta_t MucGetShootSpeed( void )//读取工作状态
{
    return shootStatus;
}



//发射机构初始化
void MucShootInit( void )
{
	BigShootInit();
	SmallShootInit();
}

//大小摩擦轮控制函数:同时开启和关闭
void Shoot_Loop_Task()
{
	static int CouldRun = 0;//允许打开摩擦轮标志
	static int MouseRightPressTime = 0;
	
	static int BigLeftSpeed = 0;//大摩擦轮目标速度
	static int BigRightSpeed = 0;
	
	if(CC_Condition.S2Sta == 1)//遥控器:
	{
		if(CC_Condition.S1Action == SwitchAction_3TO1)
		{
			//小摩擦轮
			if(SmallWheelSta == WheelStop)
			{ SmallWheelSta = WheelRun; }
			else
				{ SmallWheelSta = WheelStop; }
			
			//大摩擦轮
			if(BigWheelSta == WheelStop)
			{ BigWheelSta = WheelRun; }
			else
				{ BigWheelSta = WheelStop; }
		}
	}
	else if(CC_Condition.S2Sta == 3)//键盘:右键单击开启,长按SHIFT与右键关闭
		{
			if(SmallWheelSta == WheelStop && BigWheelSta == WheelStop && CouldRun == 1)//停止状态下
			{
				if(CC_Condition.MouseRightBt == PressDown )
				{ 
					SmallWheelSta = WheelRun;
					BigWheelSta = WheelRun;
				}
			}
			else//转动状态下
			{
				if(CC_Condition.MouseRightBt == PressDown && CC_Condition.Key_SHIFT == PressDown)
				{ MouseRightPressTime++; }
				else
				{ MouseRightPressTime = 0; }
				
				if(MouseRightPressTime > 500)//SHIFT 和 鼠标右键长按关闭
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
		else if(CC_Condition.S2Sta == 2)//停止
		{	
		  SmallWheelSta = WheelStop;
      BigWheelSta = WheelStop;			
		}
	
	//小摩擦轮发送电流
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
	//大摩擦轮电流发送
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
	PID_Calc(&Shoot_left_speed_pid,Shoot_Left_Feedback.calc*2,BigLeftSpeed);//两个摩擦轮电机的PID计算
  PID_Calc(&Shoot_right_speed_pid,Shoot_Right_Feedback.calc*2,BigRightSpeed);
    
	//转动时间计算
	if(SmallWheelSta == WheelRun){//小
		SmallWheelRunTime ++;
	}
	else
	{
		SmallWheelRunTime = 0;
	}
	
	if(BigWheelSta == WheelRun){//大
		BigWheelRunTime ++;
	}
	else
	{
		BigWheelRunTime = 0;
	}

}
	
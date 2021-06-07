#include "muc_main_task.h"
#include "muc_timer.h"
#include "muc_led.h"
#include "muc_remote.h"
#include "muc_imu.h"
#include "muc_gimbal.h"
#include "muc_chassis.h"
#include "muc_stir_wheel.h"
#include "muc_hardware_super_capacity.h"
#include "muc_shoot_wheel.h"
#include "muc_hardware_can.h"
#include "muc_gimbal_chassis.h"
#include "muc_Shoot_Judge_Control.h"

void SnailBeginControl()
{
	if(CC_Condition.S2Sta == StaN3)
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1200);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1200);
	}

	else if(CC_Condition.S2Sta == StaN1	)
	{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1000);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1000);
	}
}

/* 程序运行时间 */
extern uint32_t mainTaskTimes;

/* 系统工作状态 */
static WorkState_t workState = PREPARE_STATE;

/* 设置当前状态 */
void SetWorkState(WorkState_t state)
{
    workState = state;
}

/* 获取当前状态 */
WorkState_t GetWorkState(void)
{
	return workState;
}
 
/* 状态检查 */
WorkState_t SystemStateCheck( void )
{
    mainTaskTimes++;
    
    if( (mainTaskTimes==1000)&&(GetWorkState()==PREPARE_STATE) )
    {
        SetWorkState(NORMAL_STATE);
        return STOP_STATE;
    }
    
    /* 遥控器断开连接 */
    if(CC_Condition.S2Sta == StaN2 )
    {
        SetWorkState(STOP_STATE);
        return STOP_STATE;
    
    } else {
        if(GetWorkState()==STOP_STATE)
        {
            SetWorkState(PREPARE_STATE);
            mainTaskTimes=0;
            MucSystemReboot();  // 系统重启
            return PREPARE_STATE;
        }
    }
    
    return GetWorkState();
}

/* 启动主任务定时器,打开统计任务时间定时器 由main函数调用*/
void TaskAllBeginRun( void )
{
	/* 启动定时器 计数功能*/
	HAL_TIM_Base_Start(&htim14);
	
	/* 启动定时器 中断功能 1ms */
	HAL_TIM_Base_Start_IT(&htim6); //
	
}

/* 弹仓控制 */
void MagazineContorl() 
{
	static int Store_Flag=0;
	if( CC_Condition.Key_R==PressDown&&Store_Flag==0)
	{
    TIM4->CCR3 =2000;
		Store_Flag=1;
	}
	if(CC_Condition.Key_Z==PressDown&&Store_Flag==1)
	{
		TIM4->CCR3 =1000;
		Store_Flag=0;
	}
}

/* 主进行函数,由定时器中断来调用  1KHz */
void TaskAllRunningLoop( void )
{ 
	static int SystemTime=0;
	WorkState_t nowSystemSta;
        
	SystemTime=(SystemTime+1)%100;
	
	/* LED指示灯 */
	LedTaskLoop();
	
	/* 遥控器数据处理任务 在系统状态检查之前执行*/
	RemoteTaskLoop();

	/*弹仓盖控制*/
	MagazineContorl();

	/* 系统状态检查 */
	nowSystemSta = SystemStateCheck();
	switch(nowSystemSta)
	{
		case NORMAL_STATE: break;
		case STOP_STATE:
					CAN_Send_Message( &hcan1, 0X1ff, 0, 0, 0, 0 );
					CAN_Send_Message( &hcan1, 0X200, 0, 0, 0, 0 );
					CAN_Send_Message(&hcan2, 0X320, 0, 0, 0, GC_STOP );
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,1000);
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,1000);
					HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
					return ;
					break;
		default: break;
	}
	
	/*snail电调初始化后启动处理 平时用不到，写个到时候方便*/
	//SnailBeginControl();
   
	//HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET); //y m 	
	
	/* 角速度计数据处理 */
	ImuAnalysisTask();

	/* 摩擦轮控制任务 */
	Shoot_Loop_Task();

	/* 拨轮电机控制任务 */
	SMControlLoop_Task();

	/* 枪口热量限制*/
	JudgementDataCheck_Task();

	/* 云台电机控制任务 */
	GMControlLoopTask(&CC_Condition,nowSystemSta);

	/* 地盘电机控制任务 */
	CMControlForChassisLoopTask();

	/* 超级电容控制函数 */
	SuperCapControlLoopTask();

	return ;
}

/* 离散斜坡函数 ： 插值使输入量变得平缓*/
double DataSlopeProcessing( double RealData, double TargetData, float IntervalData )
{
	if( (RealData - TargetData)>=IntervalData )
	{
		RealData -= IntervalData;
	} else if( (RealData - TargetData)<=(-IntervalData) )
	{
		RealData += IntervalData;	
	} else {
		RealData = TargetData;	
	}
	
	return RealData;
}
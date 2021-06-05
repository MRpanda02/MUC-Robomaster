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
#include "muc_visual_tasks.h"
#include "muc_shoot_wheel.h"

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

/* 启动主任务定时器,打开统计任务时间定时器 */
void TaskAllBeginRun( void )
{
    /* 计数功能 定时器 */
    HAL_TIM_Base_Start(&htim14);
    
    /* 定时中断 1ms */
    HAL_TIM_Base_Start_IT(&htim6);
    
}


/* 主进行函数,由定时器中断来调用  1KHz */
void TaskAllRunningLoop( void )
{   
    WorkState_t nowSystemSta;

    /* LED指示灯 */
    LedTaskLoop();
    
    /* 遥控器数据处理任务 在系统状态检查之前执行*/
    RemoteTaskLoop();
   
    /* 系统状态检查 */
    nowSystemSta = SystemStateCheck();
    switch(nowSystemSta){
        case NORMAL_STATE:
        break;
        
        case STOP_STATE:
            CAN_Send_Message( &hcan1, 0X1ff, 0, 0, 0, 0 );
            CAN_Send_Message( &hcan1, 0X200, 0, 0, 0, 0 );
				    CAN_Send_Message( &hcan2, 0X1ff, 0, 0, 0, 0 );
						CAN_Send_Message( &hcan2, 0X200, 0, 0, 0, 0 );
            CAN_Send_Message(&hcan2, 0X320, 0, 0, 0, GC_STOP );
            return ;
        break;
        
        default:
        break;
    }
    
    
    
    /* 角速度计数据处理 */
    ImuAnalysisTask();
    
    /* 摩擦轮控制任务 */
		//Shoot_Loop_Task();//摩擦轮控制:同时开关
		
    /* 拨轮电机控制任务 */
    //SMControlLoop_Task();//大波轮与大弹推弹机构
		
    /* 云台电机控制任务 */
    GMControlLoopTask();
		
    /* 地盘电机控制任务 */
    CMControlForChassisLoopTask();//发送至底盘
    //SetChassisSpeedDataFromGM();
	//CMControlOnlyChassisLoopTask();
    /* 超级电容控制函数 */
//    SuperCapControlLoopTask();//SHIFT 发送
		

    return ;
}


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


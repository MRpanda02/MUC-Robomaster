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

/* ��������ʱ�� */
extern uint32_t mainTaskTimes;
/* ϵͳ����״̬ */
static WorkState_t workState = PREPARE_STATE;



/* ���õ�ǰ״̬ */
void SetWorkState(WorkState_t state)
{
    workState = state;
}
/* ��ȡ��ǰ״̬ */
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
    
    /* ң�����Ͽ����� */
    if(CC_Condition.S2Sta == StaN2 )
    {
        SetWorkState(STOP_STATE);
        return STOP_STATE;
    
    } else {
        if(GetWorkState()==STOP_STATE)
        {
            SetWorkState(PREPARE_STATE);
            mainTaskTimes=0;
            MucSystemReboot();  // ϵͳ����
            return PREPARE_STATE;
        }
    }
    
    return GetWorkState();
}

/* ����������ʱ��,��ͳ������ʱ�䶨ʱ�� */
void TaskAllBeginRun( void )
{
    /* �������� ��ʱ�� */
    HAL_TIM_Base_Start(&htim14);
    
    /* ��ʱ�ж� 1ms */
    HAL_TIM_Base_Start_IT(&htim6);
    
}


/* �����к���,�ɶ�ʱ���ж�������  1KHz */
void TaskAllRunningLoop( void )
{   
    WorkState_t nowSystemSta;

    /* LEDָʾ�� */
    LedTaskLoop();
    
    /* ң�������ݴ������� ��ϵͳ״̬���֮ǰִ��*/
    RemoteTaskLoop();
   
    /* ϵͳ״̬��� */
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
    
    
    
    /* ���ٶȼ����ݴ��� */
    ImuAnalysisTask();
    
    /* Ħ���ֿ������� */
		//Shoot_Loop_Task();//Ħ���ֿ���:ͬʱ����
		
    /* ���ֵ���������� */
    //SMControlLoop_Task();//��������Ƶ�����
		
    /* ��̨����������� */
    GMControlLoopTask();
		
    /* ���̵���������� */
    CMControlForChassisLoopTask();//����������
    //SetChassisSpeedDataFromGM();
	//CMControlOnlyChassisLoopTask();
    /* �������ݿ��ƺ��� */
//    SuperCapControlLoopTask();//SHIFT ����
		

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


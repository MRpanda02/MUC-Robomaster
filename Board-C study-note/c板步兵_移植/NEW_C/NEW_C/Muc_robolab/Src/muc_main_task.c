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
 
/* ״̬��� */
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

/* ����������ʱ��,��ͳ������ʱ�䶨ʱ�� ��main��������*/
void TaskAllBeginRun( void )
{
	/* ������ʱ�� ��������*/
	HAL_TIM_Base_Start(&htim14);
	
	/* ������ʱ�� �жϹ��� 1ms */
	HAL_TIM_Base_Start_IT(&htim6); //
	
}

/* ���ֿ��� */
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

/* �����к���,�ɶ�ʱ���ж�������  1KHz */
void TaskAllRunningLoop( void )
{ 
	static int SystemTime=0;
	WorkState_t nowSystemSta;
        
	SystemTime=(SystemTime+1)%100;
	
	/* LEDָʾ�� */
	LedTaskLoop();
	
	/* ң�������ݴ������� ��ϵͳ״̬���֮ǰִ��*/
	RemoteTaskLoop();

	/*���ָǿ���*/
	MagazineContorl();

	/* ϵͳ״̬��� */
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
	
	/*snail�����ʼ������������ ƽʱ�ò�����д����ʱ�򷽱�*/
	//SnailBeginControl();
   
	//HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET); //y m 	
	
	/* ���ٶȼ����ݴ��� */
	ImuAnalysisTask();

	/* Ħ���ֿ������� */
	Shoot_Loop_Task();

	/* ���ֵ���������� */
	SMControlLoop_Task();

	/* ǹ����������*/
	JudgementDataCheck_Task();

	/* ��̨����������� */
	GMControlLoopTask(&CC_Condition,nowSystemSta);

	/* ���̵���������� */
	CMControlForChassisLoopTask();

	/* �������ݿ��ƺ��� */
	SuperCapControlLoopTask();

	return ;
}

/* ��ɢб�º��� �� ��ֵʹ���������ƽ��*/
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
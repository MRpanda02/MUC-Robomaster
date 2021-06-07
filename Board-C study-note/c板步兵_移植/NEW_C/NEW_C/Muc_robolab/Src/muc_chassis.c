#include "muc_chassis.h"
#include "muc_pid.h"
#include "muc_remote.h"
#include "muc_gimbal.h"
#include "muc_hardware_can.h"
#include "muc_main_task.h"
#include "muc_gimbal_chassis.h"
#include <math.h>
#include <string.h>

//�洢����CAN��������
CMReceiveTypeDef CM1_Feedback;
CMReceiveTypeDef CM2_Feedback;
CMReceiveTypeDef CM3_Feedback;
CMReceiveTypeDef CM4_Feedback;

// ����PID����
PidTypeDef CM1_speed_pid={0};
PidTypeDef CM2_speed_pid={0};
PidTypeDef CM3_speed_pid={0};
PidTypeDef CM4_speed_pid={0};

PidTypeDef CM_rotate_pid={0};
PidTypeDef CM_circle_pid={0};

static double CMRotate_angle_ecd=0;
static double CMRotate_angle_ref=0;

static int CMFrontBackSpeed=0;
static int CMLeftRightSpeed=0;


static int Chassis_Circle_Speed=0;

int Sta_To_Circle=0; //����״̬     Ĭ�Ϲر�
int Sta_To_Fllow=1;  //���̸���״̬ Ĭ�ϸ���

void MucChassisInit( void )
{
    
    PID_Init(&CM_rotate_pid,CMRotatePid_KP,CMRotatePid_KI ,CMRotatePid_KD,0,700,0,0,700);
	PID_Init(&CM_circle_pid,CMCirclePid_KP,CMCirclePid_KI ,CMCirclePid_KD,0,700,0,0,700);
	
	PID_Init(&CM1_speed_pid,CMSpeed_Pid_NORMAL_KP,CMSpeed_Pid_NORMAL_KI, CMSpeed_Pid_NORMAL_KD,0,12000,0,0,12000);
	
    memcpy(&CM2_speed_pid,&CM1_speed_pid,sizeof(CM1_speed_pid));
    memcpy(&CM3_speed_pid,&CM1_speed_pid,sizeof(CM1_speed_pid));
    memcpy(&CM4_speed_pid,&CM1_speed_pid,sizeof(CM1_speed_pid));
}

/***********************���̵�����ٶȷ�������**************************/
void ChassisDealDataFromCan(CMReceiveTypeDef *Receive, uint8_t *msgData)
{
   Receive->real[Receive->count]=(msgData[2]<<8)|msgData[3];
	Receive->count++;
	if(	Receive->count>3)
	{
		Receive->calc=(Receive->real[0]+Receive->real[1]+Receive->real[2]+Receive->real[3])/4;
		Receive->count=0;
	}
}

int Flag_Key_WASD=0;

/*���̼�����괦��*/
void ChassisDataAnalysisKey(void)
{
		//static int Flag_Key_WASD=0;
		static int Flag_Circle=0;
		static uint16_t SHIFT_press_time=0;

		int FrontBackTemp=0;
		int LeftRightTemp=0;
		float Fb_Slope;  //ǰ��˿���̶�
		float Lr_Slope;  //����˿���̶�

		if(CC_Condition.Key_W==PressDown)
		{
			if(CC_Condition.Key_SHIFT == PressDown )
			{
				SHIFT_press_time++;
				if(SHIFT_press_time<500)
				{
					FrontBackTemp = 1400;
				}
				else
				{	
					FrontBackTemp = 1400;
				}	
			}
			else
			{
				FrontBackTemp = 900;
				SHIFT_press_time=0;
			}	
		}
		else
		{
			SHIFT_press_time=0;
		}

		if(CC_Condition.Key_S==PressDown)
		{
			if(CC_Condition.Key_SHIFT == PressDown )
			{
				SHIFT_press_time++;
				if(SHIFT_press_time<500)  FrontBackTemp = -900;
				else FrontBackTemp = -1400;
			}
			else
			{
				FrontBackTemp = -900;
				SHIFT_press_time=0;
			}
		}
		else
		{
			SHIFT_press_time=0;
		}

		if(CC_Condition.Key_A==PressDown)
		{
			
			LeftRightTemp = -900;
			if(CC_Condition.Key_S==PressDown||CC_Condition.Key_W==PressDown)
			{
				LeftRightTemp *= 0.7;
				FrontBackTemp	*= 0.7;
			}
		}


		if(CC_Condition.Key_D==PressDown)
		{
		
			LeftRightTemp = 900;
			if(CC_Condition.Key_S==PressDown||CC_Condition.Key_W==PressDown)
			{
				LeftRightTemp *= 0.7;
				FrontBackTemp	*= 0.7;
			}
		}
		
		if( IMUAngleData<80||IMUAngleData>-80)
		{
			if(CC_Condition.Key_Q==PressDown)
			{ 
				CC_Condition.yaw_angle_ref -= 0.15f;	
			}
			if(CC_Condition.Key_E==PressDown)
			{
				CC_Condition.yaw_angle_ref += 0.15f;	
			}
		}

		if(CC_Condition.Key_X==PressDown)
		{
			 GMYawEncoder.round_cnt=0;
			 Sta_To_Fllow=1;
			 Sta_To_Circle =0;
			 Flag_Circle=1;			
		}
		
		if(CC_Condition.Key_C==PressDown)
		{		   
			Sta_To_Fllow=0;
			Sta_To_Circle =1;
			Flag_Circle=0;
		}
			 

		if( MyAbs(CC_Condition.FrontBack)<=900 ){ Fb_Slope = 8.0f; }
		else{ Fb_Slope = 2.0f; }
		
		if( MyAbs(CC_Condition.LeftRight)<=900 ) { Lr_Slope = 8.0f; }
		else{ Lr_Slope = 2.0f; }
		
		if(FrontBackTemp==0){ Fb_Slope = 8.0; }
		if(LeftRightTemp==0){ Lr_Slope = 8.0; }
			
		CC_Condition.FrontBack =  (int)DataSlopeProcessing((double)CC_Condition.FrontBack,(double)FrontBackTemp,Fb_Slope);
		CC_Condition.LeftRight =  (int)DataSlopeProcessing((double)CC_Condition.LeftRight,(double)LeftRightTemp,Lr_Slope);

}
/* ����ң�������� */
void ChassisDataAnalysisRomote(void)
{
	
    Sta_To_Circle=0;    //Circle��ת״̬λ
    Sta_To_Fllow=1;  //���̸���״̬λ
	
    CC_Condition.FrontBack = (int)(g_RcRawData.ch2*1.36);
    CC_Condition.LeftRight = (int)(g_RcRawData.ch1*1.36);	

}

/* ���̿��Ʒ�ʽѡ�� */
void ChassisDataAnalysisWay(void)
{
		Flag_Key_WASD=0;
		if(g_RcRawData.s2==1)  ChassisDataAnalysisRomote();

		else if(g_RcRawData.s2==3) ChassisDataAnalysisKey();

		if( MyAbs(CC_Condition.FrontBack)>10 )
		{
			Flag_Key_WASD=1;
		}
		if( MyAbs(CC_Condition.FrontBack)>10 )
		{
			Flag_Key_WASD=1;
		}
}

/* ���õ����˶��ٶ� */
void SetChassisSpeedTask( void )
{
		/* ����Ŀ���ٶ� */
		if( Flag_Key_WASD==1 )
		{
			Chassis_Circle_Speed = 300;	
		}
		else 
		{	
			Chassis_Circle_Speed = 800;
		}
	
    /* ��������̨�ļн� Ŀ��Ƕ� */
    CMRotate_angle_ref = 0.0;
		
    /* ������̨����ĽǶȷ�����Ϣ */
    CMRotate_angle_ecd = GMYawEncoder.ecd_angle;
	
    CMFrontBackSpeed = (int)CC_Condition.FrontBack;
    CMLeftRightSpeed = (int)CC_Condition.LeftRight;
 	
}

/**
  * ���̵����������  û����
  */
void CMControlLoopTask(void)
{
	int CM1_target_speed=0;
	int CM2_target_speed=0;
	int CM3_target_speed=0;
	int CM4_target_speed=0;

	int FrontBackTemp=0;
    int LeftRightTemp=0;
	
	float cosData;
	float sinData;
	
	static int CM1_real_speed=0;
	static int CM2_real_speed=0;
	static int CM3_real_speed=0;
	static int CM4_real_speed=0;
    
	ChassisDataAnalysisWay();
    SetChassisSpeedTask();

	PID_Calc(&CM_rotate_pid,CMRotate_angle_ecd,CMRotate_angle_ref);
	
	 
	cosData = cos(CMRotate_angle_ecd*0.0174533f);
	sinData = sin(CMRotate_angle_ecd*0.0174533f);
	
	FrontBackTemp = (int)CMFrontBackSpeed*cosData;
	LeftRightTemp = (int)(-CMFrontBackSpeed*sinData);

	LeftRightTemp += (int)CMLeftRightSpeed*cosData;
	FrontBackTemp += (int)CMLeftRightSpeed*sinData;

	// ��ȡң�����з���������  ���������ĸ������ʹ�ĸ������ȡ��Ӧ���е�ת�٣�
	CM1_target_speed = (-LeftRightTemp - FrontBackTemp) * KP_Remote + (int)(CM_rotate_pid.output*KP_Rotate*Sta_To_Fllow)+(int)1000*Sta_To_Circle;
	CM2_target_speed = (-LeftRightTemp + FrontBackTemp) * KP_Remote + (int)(CM_rotate_pid.output*KP_Rotate*Sta_To_Fllow)+(int)1000*Sta_To_Circle;
	CM3_target_speed = (LeftRightTemp  + FrontBackTemp) * KP_Remote + (int)(CM_rotate_pid.output*KP_Rotate*Sta_To_Fllow)+(int)1000*Sta_To_Circle;
	CM4_target_speed = (LeftRightTemp  - FrontBackTemp) * KP_Remote + (int)(CM_rotate_pid.output*KP_Rotate*Sta_To_Fllow)+(int)1000*Sta_To_Circle;
	
	CM1_real_speed =  (int)DataSlopeProcessing((double)CM1_real_speed,(double)CM1_target_speed,20.0f);
	CM2_real_speed =  (int)DataSlopeProcessing((double)CM2_real_speed,(double)CM2_target_speed,20.0f);
	CM3_real_speed =  (int)DataSlopeProcessing((double)CM3_real_speed,(double)CM3_target_speed,20.0f);
	CM4_real_speed =  (int)DataSlopeProcessing((double)CM4_real_speed,(double)CM4_target_speed,20.0f);
	
    PID_Calc(&CM1_speed_pid,CM1_Feedback.real[0],CM1_real_speed);//�ĸ����̵����PID����
	PID_Calc(&CM2_speed_pid,CM2_Feedback.real[0],CM2_real_speed);
	PID_Calc(&CM3_speed_pid,CM3_Feedback.real[0],CM3_real_speed);
	PID_Calc(&CM4_speed_pid,CM4_Feedback.real[0],CM4_real_speed);
    
    if( GetWorkState()==STOP_STATE )
    {
        CAN_Send_Message(&hcan1, 0X200, 0, 0, 0, 0 );
    } else if( GetWorkState()==PREPARE_STATE ){
        /* ����������� */
        CAN_Send_Message(&hcan2, 0X200,CM1_speed_pid.output/2,CM2_speed_pid.output/2,CM3_speed_pid.output/2,CM4_speed_pid.output/2);
    }else{
        /* ����������� */
        CAN_Send_Message(&hcan2, 0X200,CM1_speed_pid.output,CM2_speed_pid.output,CM3_speed_pid.output,CM4_speed_pid.output);
    }
}

 /**
  * ���̵����������  
  */
void CMControlForChassisLoopTask(void)
{
		int FrontBackTemp=0;
		int LeftRightTemp=0;
		int RotateTemp = 0;
		
		float cosData;
		float sinData;
		
		ChassisDataAnalysisWay();

		SetChassisSpeedTask();
		
		PID_Calc(&CM_rotate_pid,CMRotate_angle_ecd,CMRotate_angle_ref);
			
		cosData = cos(CMRotate_angle_ecd*0.0174533f);
		sinData = sin(CMRotate_angle_ecd*0.0174533f);

		FrontBackTemp = (int)( CMFrontBackSpeed*cosData);
		LeftRightTemp = (int)(-CMFrontBackSpeed*sinData);

		LeftRightTemp += (int)(CMLeftRightSpeed*cosData);
		FrontBackTemp += (int)(CMLeftRightSpeed*sinData);
		
		RotateTemp = CM_rotate_pid.output*Sta_To_Fllow+Chassis_Circle_Speed*Sta_To_Circle;

		if( GetWorkState()==STOP_STATE )
		{
				CAN_Send_Message(&hcan2, 0X320, 0, 0, 0, GC_STOP );
		}
		else if( GetWorkState()==PREPARE_STATE )
		{
			/* ����������� */
			CAN_Send_Message(&hcan2, 0X320, 0, 0, 0, GC_NORMAL );
		}
		else
		{
			/* ����������� */
			CAN_Send_Message(&hcan2, 0X320,FrontBackTemp,LeftRightTemp,RotateTemp,GC_NORMAL);
			//CAN_Send_Message(&hcan2, 0X320,0X1234,0X5678,0X9ABC,0);
		}
}



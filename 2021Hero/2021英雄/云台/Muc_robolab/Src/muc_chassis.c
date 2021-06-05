#include "muc_chassis.h"
#include "muc_pid.h"
#include "muc_remote.h"
#include "muc_gimbal.h"
#include "muc_hardware_can.h"
#include "muc_main_task.h"
#include "muc_gimbal_chassis.h"
#include <math.h>
#include <string.h>
#include "muc_shoot_wheel.h"

//存储处理CAN反馈数据
CMReceiveTypeDef CM1_Feedback;
CMReceiveTypeDef CM2_Feedback;
CMReceiveTypeDef CM3_Feedback;
CMReceiveTypeDef CM4_Feedback;

// 地盘PID参数
PidTypeDef CM1_speed_pid={0};
PidTypeDef CM2_speed_pid={0};
PidTypeDef CM3_speed_pid={0};
PidTypeDef CM4_speed_pid={0};

PidTypeDef CM_rotate_pid={0};

int Sta_To_Circle=0; //自旋状态     默认关闭

//底盘初始化
void MucChassisInit( void )
{
    
    PID_Init(&CM_rotate_pid,CMRotatePid_KP,CMRotatePid_KI ,CMRotatePid_KD,0,700,0,0,700);
    
	//四个轮子pid初始化
	  PID_Init(&CM1_speed_pid,CMSpeed_Pid_NORMAL_KP,CMSpeed_Pid_NORMAL_KI, \
				CMSpeed_Pid_NORMAL_KD,0,12000,0,0,12000);
    memcpy(&CM2_speed_pid,&CM1_speed_pid,sizeof(CM1_speed_pid));
    memcpy(&CM3_speed_pid,&CM1_speed_pid,sizeof(CM1_speed_pid));
    memcpy(&CM4_speed_pid,&CM1_speed_pid,sizeof(CM1_speed_pid));
}

/***********************底盘电机的速度反馈处理**************************/
void Speed_Data_deal(CMReceiveTypeDef *Receive,uint8_t * msgData)
{
	Receive->real[Receive->count]=(msgData[2]<<8)|msgData[3];
	Receive->count++;
	if(	Receive->count>3)
	{
		Receive->calc=(Receive->real[0]+Receive->real[1]+Receive->real[2]+Receive->real[3])/4;
		Receive->count=0;
	}
}

void ChassisDealDataFromCan(CMReceiveTypeDef *v, uint8_t *msgData)
{
    Speed_Data_deal(v,msgData); 
}

static double CMRotate_angle_ecd=0;
static double CMRotate_angle_ref=0;

static int CMFrontBackSpeed=0;
static int CMLeftRightSpeed=0;





int Sta_To_Fllow=1;  //底盘跟随状态位
int Speed360=0;
void ChassisDataAnalysisKey(void)
{
	
 
static uint16_t SHIFT_press_time=0;
	
int FrontBackTemp=0;
int LeftRightTemp=0;
float Fb_Slope;  //前后丝滑程度
float Lr_Slope;  //左右丝滑程度
	
//A,S,D,W 移动
		if(CC_Condition.Key_W==PressDown)
		{
			
			if(CC_Condition.Key_SHIFT == PressDown )
			{
		        SHIFT_press_time++;
//	         if(SHIFT_press_time<500)
				FrontBackTemp = 1500;//6000;
//			else
//				FrontBackTemp = 6000;
				
			}else if( CC_Condition.Key_CTRL == PressDown ){
				FrontBackTemp = 500;
				SHIFT_press_time=0;
			}else
			{
				FrontBackTemp = 900;
				SHIFT_press_time=0;
			}	
		}
		else
		{SHIFT_press_time=0;}

		if(CC_Condition.Key_S==PressDown)
		{
			if(CC_Condition.Key_SHIFT == PressDown )
			{
				SHIFT_press_time++;
				if(SHIFT_press_time<1000)
					FrontBackTemp = -900;
				else
				FrontBackTemp = -1500;
			}else if( CC_Condition.Key_CTRL == PressDown ){
				FrontBackTemp = -700;
				SHIFT_press_time=0;
			}else
			{
				FrontBackTemp = -900;
				SHIFT_press_time=0;
			}
		}
		else
		{SHIFT_press_time=0;}

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
		
		
		
//Q,E 转向
		if( Mpu6500AngleData<80||Mpu6500AngleData>-80)
		{
			if(CC_Condition.Key_Q==PressDown)
			{
				CC_Condition.yaw_angle_ref -= 0.08;//0.05;
			}
			if(CC_Condition.Key_E==PressDown)
			{
				CC_Condition.yaw_angle_ref += 0.08;//0.05;	
			}
		}
		 
  	    
//斜坡丝滑程度处理
    if(MyAbs(CC_Condition.FrontBack)<=900){
		   Fb_Slope = 8.0f;
		}
		else{
		   Fb_Slope = 2.0f;
		}
		
		if(MyAbs(CC_Condition.LeftRight)<=900){
		   Lr_Slope = 8.0f;
		}
		else{
		   Lr_Slope = 2.0f;
		}
		
		if(FrontBackTemp==0){
		   Fb_Slope = 8.0;
		}
		
		if(LeftRightTemp==0){
		   Lr_Slope = 8.0;
		}
		
//键盘 FrontBack,LeftRight斜坡函数处理
CC_Condition.FrontBack =  (int)DataSlopeProcessing((double)CC_Condition.FrontBack,(double)FrontBackTemp,Fb_Slope);
CC_Condition.LeftRight =  (int)DataSlopeProcessing((double)CC_Condition.LeftRight,(double)LeftRightTemp,Lr_Slope);
}



//遥控器通道值等设置
void ChassisDataAnalysisRomote(void)
{
	
    Sta_To_Fllow=1;  //底盘跟随状态位
    CC_Condition.FrontBack = (int)(g_RcRawData.ch2*1.6);
    CC_Condition.LeftRight = (int)(g_RcRawData.ch1*1.6);	
}


void ChassisDataAnalysisWay(void)
{
if(g_RcRawData.s2==1)  ChassisDataAnalysisRomote();//遥控器操作


else if(g_RcRawData.s2==3) ChassisDataAnalysisKey();//键盘操作

}


void SetChassisSpeedTask( void )
{
    /* 设置云台电机的角度反馈信息 */
    CMRotate_angle_ecd = GMYawEncoder.ecd_angle;
    
    /* 地盘与云台的夹角 目标角度 */
    CMRotate_angle_ref = 0.0;
    
    CMFrontBackSpeed = -(int)CC_Condition.LeftRight;
    CMLeftRightSpeed = -(int)CC_Condition.FrontBack;
    
}

 /**
  * 发送至底盘信息处理
  */

static int CM1_real_speed=0;
static int CM2_real_speed=0;
static int CM3_real_speed=0;
static int CM4_real_speed=0;
	int CM1_target_speed=0;
	int CM2_target_speed=0;
	int CM3_target_speed=0;
	int CM4_target_speed=0;
void CMControlForChassisLoopTask(void)
{
#define Lenthrate 355/295 //后轮距中心的距离比前轮距中心的距离
	int FrontBackTemp=0;
	int LeftRightTemp=0;
	float RotateTemp = 0;
	
	float cosData;
	float sinData;
	
	static int32_t runTimes=0;
	
	runTimes++;
	runTimes = runTimes%30000;
	
  static int EludeFlag = 0;//扭腰标志
  static int ShakeFlag = 0;//抖弹标志
    
    ChassisDataAnalysisWay();
    SetChassisSpeedTask();

	CMRotate_angle_ref=CMRotate_angle_ref* Sta_To_Fllow;
	PID_Calc(&CM_rotate_pid,CMRotate_angle_ecd,CMRotate_angle_ref);
	
	//消抖
//	if(MyAbs(CMRotate_angle_ecd) < 5)
//	{
//		CMRotate_angle_ecd = 0;
//		Sta_To_Fllow = 0;
//	}
//	else{
//		Sta_To_Fllow = 1;
//	}

    if(CC_Condition.S1Sta == StaN3)
	{
		Sta_To_Fllow = 0;
		CMRotate_angle_ecd=0;
	}
	

	//CMRotate_angle_ecd = 0;			//关闭跟踪关键（1）
	
	cosData = cos(CMRotate_angle_ecd*0.0174533f);
	sinData = sin(CMRotate_angle_ecd*0.0174533f);

	FrontBackTemp = (int)(CMFrontBackSpeed*cosData);
	LeftRightTemp = (int)(-CMFrontBackSpeed*sinData);

	LeftRightTemp += (int)(CMLeftRightSpeed*cosData);
	FrontBackTemp += (int)(CMLeftRightSpeed*sinData);
	
	//Sta_To_Fllow = 0;				//关闭跟踪关键（2）
	CM1_target_speed = (-LeftRightTemp - FrontBackTemp) * KP_Remote  + (int)(CM_rotate_pid.output*KP_Rotate*Sta_To_Fllow)+(int)1000*Sta_To_Circle;
	CM2_target_speed = (-LeftRightTemp + FrontBackTemp) * KP_Remote  + (int)(CM_rotate_pid.output*KP_Rotate*Sta_To_Fllow)*Lenthrate+(int)1000*Sta_To_Circle;
	CM3_target_speed = (LeftRightTemp  + FrontBackTemp) * KP_Remote  + (int)(CM_rotate_pid.output*KP_Rotate*Sta_To_Fllow)*Lenthrate+(int)1000*Sta_To_Circle;
	CM4_target_speed = (LeftRightTemp  - FrontBackTemp) * KP_Remote  + (int)(CM_rotate_pid.output*KP_Rotate*Sta_To_Fllow)+(int)1000*Sta_To_Circle;
	
	CM1_real_speed =  (int)DataSlopeProcessing((double)CM1_real_speed,(double)CM1_target_speed,20.0f);
	CM2_real_speed =  (int)DataSlopeProcessing((double)CM2_real_speed,(double)CM2_target_speed,20.0f);
	CM3_real_speed =  (int)DataSlopeProcessing((double)CM3_real_speed,(double)CM3_target_speed,20.0f);
	CM4_real_speed =  (int)DataSlopeProcessing((double)CM4_real_speed,(double)CM4_target_speed,20.0f);
	
    PID_Calc(&CM1_speed_pid,CM1_Feedback.real[0],CM1_real_speed);//四个底盘电机的PID计算
	PID_Calc(&CM2_speed_pid,CM2_Feedback.real[0],CM2_real_speed);
	PID_Calc(&CM3_speed_pid,CM3_Feedback.real[0],CM3_real_speed);
	PID_Calc(&CM4_speed_pid,CM4_Feedback.real[0],CM4_real_speed);
	RotateTemp = GMYawEncoder.ecd_angle;

////遥控器测试
	if(CC_Condition.S1Action == SwitchAction_2TO3 && BigWheelSta == WheelStop)
	{
		if(EludeFlag == 0)
	    EludeFlag = 1; 
		else
			EludeFlag = 0;
	}

//键盘F开扭腰
	if(EludeFlag == 0 && CC_Condition.Key_F == PressDown)
	{//F开启
		EludeFlag = 1;
	}
	else if((EludeFlag != 0) && (CC_Condition.Key_A  == PressDown || CC_Condition.Key_S  == PressDown || CC_Condition.Key_D  == PressDown || CC_Condition.Key_W  == PressDown))
	{//移动就关闭
		EludeFlag = 0;
	}
	if(EludeFlag != 0)
	{
		static float RotateTemp_real = 0;
		static float RotateTemp_target = 0;
		if(GMYawEncoder.ecd_angle >= 5)
		{
			EludeFlag = 1;
		}
		else if(GMYawEncoder.ecd_angle <= -5)
		{
			EludeFlag = -1;
		}
		RotateTemp_real = 30 * EludeFlag;//35
		RotateTemp_target=(float)DataSlopeProcessing((double)RotateTemp_target,(double)RotateTemp_real,0.6f);
		RotateTemp = RotateTemp_target;
		
		FrontBackTemp = 30;//扭腰中心不在底盘中心,消除向后的速度分量
	}  
	
// can2 发送信息到底盘
	if(runTimes%2 == 1){
    if( GetWorkState()==STOP_STATE )
    {
        //CAN_Send_Message(&hcan2, 0X320, 0, 0, 0, GC_STOP );
		CAN_Send_Message(&hcan2, 0X200, 0, 0, 0, 0 );
		
    } else if( GetWorkState()==PREPARE_STATE ){
        /* 电机电流发送 */
		
		CAN_Send_Message(&hcan2, 0X200,CM1_speed_pid.output/2,CM2_speed_pid.output/2,CM3_speed_pid.output/2,CM4_speed_pid.output/2);

    }else{
        /* 电机电流发送 */
		CAN_Send_Message(&hcan2, 0X200,CM1_speed_pid.output,CM2_speed_pid.output,CM3_speed_pid.output,CM4_speed_pid.output);

		}
	}	
}





/**
  * 底盘电机控制任务
  * 只是地盘控制时
  */
int FrontBackGMData=0;
int LeftRightGMData=0;
int RotateGMData=0;

int ShiftRotate = 0;//Shift按下时快速跟随

void SetChassisSpeedDataFromGM( void )
{
    FrontBackGMData = g_chassisDataFromGM.rawFrontBack;
    LeftRightGMData = g_chassisDataFromGM.rawLeftRight;
    RotateGMData = g_chassisDataFromGM.rawRotate;
}

void CMControlOnlyChassisLoopTask(void)
{
#define Lenthrate 355/295 //后轮距中心的距离比前轮距中心的距离
	int CM1_target_speed=0;
	int CM2_target_speed=0;
	int CM3_target_speed=0;
	int CM4_target_speed=0;

	int FrontBackTemp=0;
    int LeftRightTemp=0;
    int RotateTemp=0;
	
	static int CM1_real_speed=0;
	static int CM2_real_speed=0;
	static int CM3_real_speed=0;
	static int CM4_real_speed=0;
  static int CM_Rote_speed=0;
	static int lastRotateTemp = 0;
	static uint32_t RotateTime = 0;
    SetChassisSpeedDataFromGM();
		 
	FrontBackTemp = FrontBackGMData;
	LeftRightTemp = LeftRightGMData;
	RotateTemp = RotateGMData;

//跟随处理
	static uint8_t flag = 0;
	if(MyAbs(RotateTemp) >= 15 && flag == 0)
	{
		flag = 1;
	}
	if(MyAbs(RotateTemp) >= 15 && flag == 1)
	{
		CM_rotate_pid.Kp = (double)(MyAbs(RotateTemp)/CM_rotate_pid.max_input) * (CM_rotate_pid_Kp_Max-CM_rotate_pid_Kp_Min) + CM_rotate_pid_Kp_Min;
	}
	else if(MyAbs(RotateTemp) < 10 && flag == 1)
	{
		flag = 0;
		CM_rotate_pid.Kp = (double)CM_rotate_pid_Kp_Max;
	}
	
	if(CM_rotate_pid.Kp < 0) CM_rotate_pid.Kp = 0;
	
	PID_Calc(&CM_rotate_pid,RotateTemp,0);
	
	CM_Rote_speed = CM_rotate_pid.output * 0.8;

//Shift快速跟随
	if(RotateTemp >= 20)
		{ ShiftRotate = -3000; }//2000
	else if(RotateTemp <= -20)
		{ ShiftRotate = 3000; }
	else
		{ ShiftRotate = 0; }
	
	// 获取云台发来的数据  （解析到四个电机，使四个电机获取到应该有的转速）
	CM1_target_speed = (LeftRightTemp + FrontBackTemp) * KP_Remote +  (CM_Rote_speed) + ShiftRotate;
	CM2_target_speed = (-LeftRightTemp + FrontBackTemp) * KP_Remote + (CM_Rote_speed)*Lenthrate + ShiftRotate;
	CM3_target_speed = (-LeftRightTemp  - FrontBackTemp) * KP_Remote +(CM_Rote_speed)*Lenthrate + ShiftRotate;
	CM4_target_speed = (LeftRightTemp  - FrontBackTemp) * KP_Remote + (CM_Rote_speed) + ShiftRotate;
	 
	CM1_real_speed =  (int)DataSlopeProcessing((double)CM1_real_speed,(double)CM1_target_speed,5000.0f); //1000
	CM2_real_speed =  (int)DataSlopeProcessing((double)CM2_real_speed,(double)CM2_target_speed,5000.0f);
	CM3_real_speed =  (int)DataSlopeProcessing((double)CM3_real_speed,(double)CM3_target_speed,5000.0f);
	CM4_real_speed =  (int)DataSlopeProcessing((double)CM4_real_speed,(double)CM4_target_speed,5000.0f);

	//owerLoopControl(g_refereeSystemReceMesg.power_heat_data.chassis_power);

	PID_Calc(&CM1_speed_pid,CM1_Feedback.real[0],CM1_real_speed);//四个底盘电机的PID计算
	PID_Calc(&CM2_speed_pid,CM2_Feedback.real[0],CM2_real_speed);
	PID_Calc(&CM3_speed_pid,CM3_Feedback.real[0],CM3_real_speed);
	PID_Calc(&CM4_speed_pid,CM4_Feedback.real[0],CM4_real_speed);
		
  if( GetWorkState()==PREPARE_STATE ){
        /* 电机电流发送 */
        CAN_Send_Message(&hcan2, 0X200,CM1_speed_pid.output/2,CM2_speed_pid.output/2,CM3_speed_pid.output/2,CM4_speed_pid.output/2);
    }else{
        /* 电机电流发送 */
        CAN_Send_Message(&hcan2, 0X200,CM1_speed_pid.output,CM2_speed_pid.output,CM3_speed_pid.output,CM4_speed_pid.output);
    }
		if( GetWorkState()==STOP_STATE )
    {
        CAN_Send_Message(&hcan2, 0X200, 0, 0, 0, 0 );
    }
		
		lastRotateTemp = RotateTemp;
}













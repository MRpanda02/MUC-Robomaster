#include "muc_gimbal.h"
#include "muc_main_task.h"
#include "muc_pid.h"
#include "muc_remote.h"
#include "muc_imu.h"
#include "muc_hardware_can.h"
#include "muc_stmflash.h"
#include "muc_hardware_beep.h"
#include "muc_hardware_key.h"
#include "muc_stir_wheel.h"
#include "muc_auto_shoot.h"

/*鼠标控制 配置参数*/
#define MOUSE_PITCH_ANGLE_NORMAL		0.0012//0.0012f
#define MOUSE_YAW_ANGLE_NORMAL 		0.002//0.0002

#define MOUSE_PITCH_ANGLE_CONTROL		0.0005//0.001f
#define MOUSE_YAW_ANGLE_CONTROL 		0.001f//0.001f

/*遥控器控制 配置参数*/
#define REMOTE_PITCH_ANGLE		0.005f//0.001f
#define REMOTE_YAW_ANGLE 		0.0002f


int16_t GimbalYawMidOffset = 4100;
int16_t GimbalPitchMidOffset = 1111;

PidTypeDef Yaw_speed_pid={0};
PidTypeDef Yaw_position_pid={0};
PidTypeDef Pitch_speed_pid={0};
PidTypeDef Pitch_position_pid={0};

double pitch_angle_pos_ref = 0.0f;  // 目标角度位置
double yaw_angle_pos_ref = 0.0f;

double pitch_angle_pos_ecd = 0.0f;
double yaw_angle_pos_ecd = 0.0f;

double pitch_angle_vel_ecd = 0.0f;
double yaw_angle_vel_ecd = 0.0f;

volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};

/*
***********************************************************************************************
*Name          :EncoderProcess
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
*
*
***********************************************************************************************
*/
void EncoderProcess(volatile Encoder *v, uint8_t *msgData)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msgData[0]<<8)|msgData[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>7500)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}


	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}

void GimbalDealDataFromCan(volatile Encoder *v, uint8_t *msgData)
{
    EncoderProcess(v,msgData); 
    /* 下面的一段代码好像没什么用 先注释了 */
//    if(GetWorkState() == PREPARE_STATE)
//    {
//        if((v->ecd_bias - v->raw_value) <-4000)
//        {
//            v->ecd_bias = GimbalPitchMidOffset + 8192;
//        }
//        else if((v->ecd_bias - v->raw_value) > 4000)
//        {
//            v->ecd_bias = GimbalPitchMidOffset - 8192;
//        }
//    }
}
uint64_t see112;
/* 更新Gimbal的误差数据，如果没有有效数据，则会提示更新 */
void UpdateGimbalGyroCaliData( void )
{
    int i;
    uint64_t gimbalPosMidTmp[2];
    if(GetCaliDataSta(GIMBAL_CALI_DATA))
    {
        // 数据有效
        GetGimbalCaliData(&GimbalPitchMidOffset,&GimbalYawMidOffset);
        
    }else{
        
        /* 重新校准 按下白色按钮开始校准 */
        bibiSing(6); /* 报错提示 */
        while(!GetKeyPressUpAction())
        {
            HAL_Delay(100);
        }
        bibiSing(2);
        
        HAL_Delay(1000);
        HAL_Delay(1000);
        HAL_Delay(1000);
        
        bibiSing(1);
        /* 校准角速度误差 */
        gimbalPosMidTmp[0]=0;
        gimbalPosMidTmp[1]=0;
        for( i=0; i<50; i++ )
        {
            gimbalPosMidTmp[0] += GMPitchEncoder.raw_value;
            gimbalPosMidTmp[1] += GMYawEncoder.raw_value;
            HAL_Delay(5);
        }
        /* 完成校准 存储数据 */
        UpdateConfigParamsForGimbal(gimbalPosMidTmp[0]/50,gimbalPosMidTmp[1]/50);
        see112= gimbalPosMidTmp[0];
        GetGimbalCaliData(&GimbalPitchMidOffset,&GimbalYawMidOffset);
        
        bibiSing(2);
    }
    
}
/* 云台初始化函数 */
void GimbalInit( void )
{
	/* 初始化PID参数 */
//	PID_Init(&Yaw_speed_pid,YawSpeedPid_KP, YawSpeedPid_KI, YawSpeedPid_KD,0,29000,0,0,27500);
//	PID_Init(&Yaw_position_pid,YawPositionPid_KP,YawPositionPid_KI, YawPositionPid_KD,0,1000,0,0,0x7ffffff);
	PID_Init(&Yaw_speed_pid,-70,0,-0.005,0,29000,0,0,27500);//-150		//-60
	PID_Init(&Yaw_position_pid,10,0,0,0,1000,0,0,0x7ffffff);//16.5 16
	
	//PID_Init(&Pitch_speed_pid,PitchSpeedPid_KP,PitchSpeedPid_KI, PitchSpeedPid_KD,0,29000,0,1000,27500);//kp150 pd500
	//PID_Init(&Pitch_position_pid,PitchPositionPid_KP,PitchPositionPid_KI, PitchPositionPid_KD,0,1000,0,0,0x7ffffff);
  //调试用
	PID_Init(&Pitch_speed_pid,-40,0, 0,0,29000,0,1000,27500);//-30,-1, 0,0,29000,0,1000,27500
	PID_Init(&Pitch_position_pid,-1,0,-1.2,0,1000,0,0,27500);//-1,0, -1.2,0,1000,0,0,0x7ffffff
    //-0.5,-0.7
    /* 加载云台中间值的参数 */
    UpdateGimbalGyroCaliData();
    
    /* 将云台中间值的参数赋值给云台电机的编码器相关变量 */
	GMPitchEncoder.ecd_bias = GimbalPitchMidOffset;
	GMYawEncoder.ecd_bias = GimbalYawMidOffset;	
}



/* 设置云台角度的工作任务 */


/* 遥控器状态数据处理 */
void Analysis_Angel_Remote(void)
{				
	
	CC_Condition.pitch_angle_ref -= (g_RcRawData.ch4)*REMOTE_PITCH_ANGLE;
	CC_Condition.yaw_angle_ref  += ((g_RcRawData.ch3)*REMOTE_YAW_ANGLE);
}


/* 键盘状态数据处理 */
uint32_t Turn_180_TaskTimes = 0;//计时
uint32_t Key_X_PressDown_Time = 0;//X键按下时间
int Turn180Flag=0;
void Analysis_Angel_Key(void)
{
    static int Vision_Task=0;
//	  static uint32_t Turn_180_TaskTimes;//计时
//	  static uint32_t Key_X_PressDown_Time;//X键按下时间
//    static int Turn180Flag=0;

	Turn_180_TaskTimes ++;//计时
    if(CC_Condition.Key_CTRL == PressDown)
    {
        CC_Condition.pitch_angle_ref -= (float)( g_RcRawData.mouse.y*MOUSE_PITCH_ANGLE_CONTROL);

        if( Mpu6500AngleData<80||Mpu6500AngleData>-80)
        {
            CC_Condition.yaw_angle_ref += (float)( g_RcRawData.mouse.x*MOUSE_YAW_ANGLE_CONTROL);//+=	
        }
    }else{

        CC_Condition.pitch_angle_ref -= (float)( g_RcRawData.mouse.y* MOUSE_PITCH_ANGLE_NORMAL);//+=

        if( Mpu6500AngleData<80||Mpu6500AngleData>-80)
        {
            CC_Condition.yaw_angle_ref += (float)( g_RcRawData.mouse.x* MOUSE_PITCH_ANGLE_NORMAL);
        }

    }
	
	/*视觉任务控制*/
if(CC_Condition.Key_V==PressDown)
	{
	  Vision_Task=0;
	}
   
	if(CC_Condition.Key_V==PressDown&&( CC_Condition.Key_SHIFT==PressDown||CC_Condition.Key_CTRL==PressDown) )
    {
	Vision_Task=1;
		
//		if(CC_Condition.Key_CTRL==PressDown)
//		{
//		Vision_Task_Windmill=1;
//		Vision_Task_Autoshooting=0;
//		}
//		
//		else if(CC_Condition.Key_SHIFT==PressDown)
//		{
//		Vision_Task_Autoshooting=1;
//		Vision_Task_Windmill=0;
//		}	
	}
	
	if(Vision_Task==1)
	{
    AutoShootingAndWindmill();
	}

	//180°
	if(CC_Condition.Key_X == PressDown && Turn180Flag == 0)
	{
		Key_X_PressDown_Time = Turn_180_TaskTimes;//记录按下X时刻时间
		Turn180Flag = 1;
	}
	else if(Turn180Flag == 1 && Turn_180_TaskTimes - Key_X_PressDown_Time <= 700)//时间控制 1000
	     { CC_Condition.yaw_angle_ref -= 0.40f; }
			 else
         { Turn180Flag = 0; }//完成180°
	

	
}

/* 云台数据方式选择 */
void Analysis_Way_Choose(void)
{
if(g_RcRawData.s2==1) Analysis_Angel_Remote();

else if(g_RcRawData.s2==3)  Analysis_Angel_Key();


}


void SetGimbalAngleTask( void )
{	
	
	Analysis_Way_Choose();
// 	AutoShootingAndWindmill();
//	VAL_LIMIT( CC_Condition.yaw_angle_ref, Mpu6500AngleData-20,Mpu6500AngleData+20 );//30
	//if(MyAbs(GMYawEncoder.ecd_angle)>=40) CC_Condition.yaw_angle_ref = Mpu6500AngleData;
    /* 限制数据的最大最小值 */
	
    VAL_LIMIT( CC_Condition.pitch_angle_ref,-1400,250);
    VAL_LIMIT( CC_Condition.yaw_angle_ref,Mpu6500AngleData - 90,  Mpu6500AngleData + 90);
    
    /* 设置云台电机的目标角度 */
    pitch_angle_pos_ref = CC_Condition.pitch_angle_ref; // 将遥控器的信息直接给云台控制量
    yaw_angle_pos_ref = CC_Condition.yaw_angle_ref;
    
	

    /* 设置云台电机的角度反馈信息 */
    pitch_angle_pos_ecd = GMPitchEncoder.ecd_angle;     // 云台电机的位置信息
    yaw_angle_pos_ecd = Mpu6500AngleData;               // 通过MPU6050计算出来的位置信息
  
	
    /* 设置云台电机的速度反馈信息 */
    pitch_angle_vel_ecd = MPU6500_Real_Data.Gyro_Y;
    yaw_angle_vel_ecd = -MPU6500_Real_Data.Gyro_Z;		//-
    
}

//云台电机控制任务
void GMControlLoopTask( void )
{
  /* 设置云台电机的控制信息 */
	SetGimbalAngleTask();
    
	/* 位置环 */
	PID_Calc( &Pitch_position_pid, pitch_angle_pos_ecd,pitch_angle_pos_ref );
	PID_Calc( &Yaw_position_pid, yaw_angle_pos_ecd, yaw_angle_pos_ref );
	
	
	/* 速度环 */
	PID_Calc( &Pitch_speed_pid, pitch_angle_vel_ecd, Pitch_position_pid.output );
	PID_Calc( &Yaw_speed_pid, yaw_angle_vel_ecd, Yaw_position_pid.output );
	
	//Yaw_speed_pid.output = 0;
  if( GetWorkState()==STOP_STATE )//停止
    {
        CAN_Send_Message(&hcan1, 0X1ff, 0, 0, 0,0);
				CAN_Send_Message(&hcan2, 0X1ff, 0, 0, 0,0);

    } else if( GetWorkState()==PREPARE_STATE ){//准备阶段
        /* 电机电流发送 */
			CAN_Send_Message( &hcan1, 0X1ff,0,Pitch_speed_pid.output, 0, 0 );
			CAN_Send_Message( &hcan2, 0X1ff,Yaw_speed_pid.output, 0, 0 ,0);

		}else{
        /* 电机电流发送 */
			CAN_Send_Message( &hcan1, 0X1ff,0,Pitch_speed_pid.output, 0, 0 );
			CAN_Send_Message( &hcan2, 0X1ff,Yaw_speed_pid.output, 0, 0 ,0);
		}
}


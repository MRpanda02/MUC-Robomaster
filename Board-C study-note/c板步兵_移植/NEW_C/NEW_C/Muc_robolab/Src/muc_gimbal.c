#include "muc_gimbal.h"
#include "muc_chassis.h"
#include "muc_main_task.h"


/*������ ���ò���*/
#define MOUSE_PITCH_ANGLE_NORMAL		0.125f
#define MOUSE_YAW_ANGLE_NORMAL 		    0.125f

#define MOUSE_PITCH_ANGLE_CONTROL		0.013f
#define MOUSE_YAW_ANGLE_CONTROL 		0.025f

/*ң�������� ���ò���*/
#define REMOTE_PITCH_ANGLE		0.0035f
#define REMOTE_YAW_ANGLE 		0.0035f

/*��̨PID����*/

/*λ�û�*/
#define POSITION_YAW_PID_KP		-25
#define POSITION_YAW_PID_KI		0
#define POSITION_YAW_PID_KD		0

#define POSITION_PITCH_PID_KP	20
#define POSITION_PITCH_PID_KI	0
#define POSITION_PITCH_PID_KD	0

/*�ٶȻ�*/
#define SPEED_YAW_PID_KP		80
#define SPEED_YAW_PID_KI		0
#define SPEED_YAW_PID_KD		0

#define SPEED_PITCH_PID_KP	    100
#define SPEED_PITCH_PID_KI		0
#define SPEED_PITCH_PID_KD		0

/*������*/
#define CURRENT_YAW_PID_KP		0
#define CURRENT_YAW_PID_KI		0
#define CURRENT_YAW_PID_KD		0

#define CURRENT_PITCH_PID_KP	0
#define CURRENT_PITCH_PID_KI	0
#define CURRENT_PITCH_PID_KD	0


int16_t GimbalYawMidOffset = 1728;
int16_t GimbalPitchMidOffset = 1444;


int target=0;
int output1=0;
int output2=0;
int feedback=0;

PidTypeDef Yaw_position_pid={0};
PidTypeDef Yaw_speed_pid={0};
PidTypeDef Yaw_current_pid={0};

PidTypeDef Pitch_position_pid={0};
PidTypeDef Pitch_speed_pid={0};
PidTypeDef Pitch_current_pid={0};

double pitch_angle_pos_ref = 0.0f;  // Ŀ��Ƕ�λ��
double yaw_angle_pos_ref = 0.0f;

double pitch_angle_pos_ecd = 0.0f; // �����Ƕ�ֵ
double yaw_angle_pos_ecd = 0.0f;

double pitch_angle_vel_ecd = 0.0f;   // �����ٶ�ֵ
double yaw_angle_vel_ecd = 0.0f;

double pitch_angle_cur_ecd = 0.0f;  // ��������ֵ
double yaw_angle_cur_ecd   = 0.0f;

volatile Encoder GMYawEncoder = {0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder = {0,0,0,0,0,0,0,0,0};

/************************************************************************************************
*Name          :GimbalDealDataFromCan ��EncoderProcess��
*Input         :can message
*Return        :void
*Description   :to get the initiatial encoder of the chassis motor 201 202 203 204
************************************************************************************************/
void GimbalDealDataFromCan(volatile Encoder *v, uint8_t *msgData,int16_t initData)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (msgData[0]<<8)|msgData[1];
	v->raw_current=(msgData[4]<<8)|msgData[5];
	v->feedback_speed_value=(msgData[2]<<8)|msgData[3];;     //�����ٶ�ֵ    
	v->diff = v->raw_value - v->last_raw_value;

	//���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
	if(v->diff < -7500){
	v->round_cnt++;
	v->ecd_raw_rate = v->diff + 8192;
	}
	
	else if(v->diff>7500){
	v->round_cnt--;
	v->ecd_raw_rate = v->diff- 8192;
	}		
	
	else{
		v->ecd_raw_rate = v->diff;
	}

	/* ����ϵУ׼ */ 
	if(GetWorkState() == PREPARE_STATE)
	{
		if(v->round_cnt<0)
		{
		v->round_cnt=0; 
		}
//		if((v->ecd_bias - v->raw_value) <-4000)
//		{
//				v->ecd_bias = initData + 8192;
//		}
//		else if((v->ecd_bias - v->raw_value) > 4000)
//		{
//				v->ecd_bias = initData - 8192;
//		}
  }

	//����õ������ı��������ֵ
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	
	//����õ��Ƕ�ֵ����Χ���������
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	
	if(v->buf_count == RATE_BUF_SIZE){
	v->buf_count = 0;
	}
	//�����ٶ�ƽ��ֵ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
	temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		
	// v->feedback_value=(msgData[2]<<8)|msgData[3];
}

/* ����Gimbal�����/У׼���ݣ����û����Ч���ݣ������ʾ���� */
void UpdateGimbalGyroCaliData( void )
{
	int i;
	uint64_t gimbalPosMidTmp[2];
	if(GetCaliDataSta(GIMBAL_CALI_DATA))
	{
		/* ������Ч */
		GetGimbalCaliData(&GimbalPitchMidOffset,&GimbalYawMidOffset);
	}else{
		
		/* ����У׼ ���°�ɫ��ť��ʼУ׼ */
		bibiSing(6); /* ������ʾ */
		while(!GetKeyPressUpAction())
		{
				HAL_Delay(100);
		}
		bibiSing(2);
		
		HAL_Delay(1000);
		HAL_Delay(1000);
		HAL_Delay(1000);
		
		bibiSing(1);
		/* У׼���ٶ���� */
		gimbalPosMidTmp[0]=0;
		gimbalPosMidTmp[1]=0;
		for( i=0; i<50; i++ )
		{
				gimbalPosMidTmp[0] += GMPitchEncoder.raw_value;
				gimbalPosMidTmp[1] += GMYawEncoder.raw_value;
				HAL_Delay(5);
		}
		/* ���У׼ �洢���� */
		UpdateConfigParamsForGimbal(gimbalPosMidTmp[0]/50,gimbalPosMidTmp[1]/50);
		GetGimbalCaliData(&GimbalPitchMidOffset,&GimbalYawMidOffset);
		
		bibiSing(2);
	}
}
/* ��̨��ʼ������ */
void GimbalInit( void )
{
	/* ��ʼ��PID���� */

	PID_Init(&Yaw_position_pid,-25,0,0,0,30000,0,0,0x7ffffff);
  PID_Init(&Yaw_speed_pid,0,0,0,0,30000,0,0,30000);
	PID_Init(&Yaw_current_pid ,0,0,0,0,28000,0,0,27500);

	
	PID_Init(&Pitch_position_pid,20,0,0,0,30000,0,0,30000);
	PID_Init(&Pitch_speed_pid,100,0,0,0,28000,0,0,28000);
  PID_Init(&Pitch_current_pid,0,0,0,0,28000,0,0,28000);

	/* ������̨�м�ֵ�Ĳ��� У׼ֵ*/
	UpdateGimbalGyroCaliData();
	
	/* ����̨�м�ֵ�Ĳ�����ֵ����̨����ı�������ر��� */
	GMPitchEncoder.ecd_bias = GimbalPitchMidOffset;
	GMYawEncoder.ecd_bias = GimbalYawMidOffset;	
}

/* ������̨�ǶȵĹ������� */
/* ��̨���ݷ�ʽѡ�� */
void Analysis_Way_Choose(CC_TypeDef *GimbalRC )
{
	if(GimbalRC->S2Sta==1) 
	{	
		GimbalRC->pitch_angle_ref += (GimbalRC->ch4)*REMOTE_PITCH_ANGLE;
		GimbalRC->yaw_angle_ref  += (GimbalRC->ch3)*REMOTE_YAW_ANGLE;

		AutoShootingAndWindmill(GimbalRC);
	}

	else if(GimbalRC->S2Sta==3)
	{
		static float RockingPosition=0;
		static int RockFlag=0;
		static int Vision_Flag=0;
		static int Vision_Task=0;
		static int V_PressDownTime=0;

		if(GimbalRC->Key_F==PressDown)
		{
			static int RockingTime=0;
			RockingTime=(RockingTime+1)%50;

			if(RockingTime<25)
			{
				GimbalRC->pitch_angle_ref=40;
			}
			else
			{
				GimbalRC->pitch_angle_ref=10;
			}
		}

		GimbalRC->pitch_angle_ref -= (float)( GimbalRC->y* MOUSE_PITCH_ANGLE_NORMAL);

		if( IMUAngleData<80||IMUAngleData>-80)
		{
			GimbalRC->yaw_angle_ref += (float)(  GimbalRC->x* MOUSE_YAW_ANGLE_NORMAL);
		}

		if(GimbalRC->MouseRightBt==PressDown)
		{
			AutoShootingAndWindmill(GimbalRC);
		}
		//��
		//	AutoShootingAndWindmill(GimbalRC);
		//	/*�Ӿ��������*/Y
		//   if(GimbalRC->Key_V==PressDown)
		//	{
		//		 V_PressDownTime++;
		//		if( V_PressDownTime>500)	
		//		{
		//			if(Vision_Task==1)
		//			{
		//			Vision_Task=0;
		//			}

		//			else if(Vision_Task==0)
		//			{
		//			Vision_Task=1;
		//			}
		//		}
		//    }

		//	if(Vision_Task==1)
		//	{
		//	AutoShootingAndWindmill(GimbalRC);
		//	}
	}
}

/* ������̨����Ŀ�����Ϣ ����λ��PID ��Ŀ��ֵ�ͷ���ֵ*/
void SetGimbalAngleTask( CC_TypeDef *GimbalRC )
{	

   VAL_LIMIT( GimbalRC->yaw_angle_ref,IMUAngleData - 60,  IMUAngleData + 60);
//	/*���Ӿ����豸û�̶�ר����λ*/
//    VAL_LIMIT( GimbalRC->yaw_angle_ref,-60,60 );
	 
  if( Sta_To_Circle ==1)
	{
		VAL_LIMIT( CC_Condition.pitch_angle_ref,9.5,60 );
	}
	else
	{
	   VAL_LIMIT( GimbalRC->pitch_angle_ref,9.5,45  );
	
	}
	
	/* ������̨�����Ŀ��Ƕ� */
	pitch_angle_pos_ref = GimbalRC->pitch_angle_ref; // ��ң��������Ϣֱ�Ӹ���̨������
	yaw_angle_pos_ref  = GimbalRC->yaw_angle_ref;

	/* ������̨����ĽǶȷ�����Ϣ */
	pitch_angle_pos_ecd = GMPitchEncoder.ecd_angle;     // ��̨�����λ����Ϣ
	yaw_angle_pos_ecd = IMUAngleData;               // ͨ��MPU6050���������λ����Ϣ

	/* ������̨������ٶȷ�����Ϣ */
	pitch_angle_vel_ecd = IMU_Real_Data.Gyro_Z;    //��̨PITCH�ٶ�
	yaw_angle_vel_ecd = - IMU_Real_Data.Gyro_Y;     //��̨YAW�ٶ�

	/* ������̨����ĵ���������Ϣ */
	pitch_angle_cur_ecd =  (double)(GMPitchEncoder.raw_current)/100;
	yaw_angle_cur_ecd   =  (double)(GMYawEncoder.raw_current)/100;
}


void GimbalDataSend(int GimbalSta,int FinalPitchSend ,int FinalYawSend )
{
	if( GimbalSta==3 )
	{
		CAN_Send_Message(&hcan1, 0X1ff, 0, 0, 0,0);
	} 
	else if( GimbalSta==0 )
	{
		/* ����������� */
		CAN_Send_Message( &hcan1, 0X1ff, (int)FinalPitchSend/2, (int) FinalYawSend/2, 0, 0 );
	}
	else
	{
    /* ����������� */
		CAN_Send_Message( &hcan1, 0X1ff, (int)FinalPitchSend, (int) FinalYawSend, StirSpeedOut, 0 );  
	}
}

int Target=0;
int Fdb=0;

/* ��̨����������� */
void GMControlLoopTask( CC_TypeDef *GimbalRC, int GimbalSta )
{
	static int time=0;
	static int ControlPeriod=0;
	ControlPeriod=( ControlPeriod+1)%5;
	time=(time+1)%2000;
	
  /* ������̨����Ŀ�����Ϣ */
	SetGimbalAngleTask(GimbalRC);
  
//  STM32f427
	/* pitch����˫������ */
	PID_Calc( &Pitch_position_pid, pitch_angle_pos_ecd,  pitch_angle_pos_ref );// pitch_angle_pos_ref
  PID_Calc( &Pitch_speed_pid, -pitch_angle_vel_ecd, Pitch_position_pid.output );

//Pitch_position_pid.output_compensation=0;
	
//   Pitch_position_pid.Kp=17;
//    PID_Calc( &Pitch_position_pid, pitch_angle_pos_ecd, pitch_angle_pos_ref  );

//    Pitch_speed_pid.Kp=2.2;
//    PID_Calc( &Pitch_speed_pid, -pitch_angle_vel_ecd, Pitch_position_pid.output_compensation );

//    Pitch_current_pid.Kp=110 ;
//    Pitch_current_pid.Ki=60;
//    PID_Calc( &Pitch_current_pid,pitch_angle_cur_ecd, Pitch_speed_pid.output_compensation);
   
	Yaw_position_pid.Kp=-16;
	Yaw_position_pid.Ki=-0.001;         
	PID_Calc( &Yaw_position_pid, yaw_angle_pos_ecd,yaw_angle_pos_ref);	
	//yaw_angle_pos_ecd,yaw_angle_pos_ref
	Yaw_speed_pid.Kp=1.2;
	PID_Calc( &Yaw_speed_pid, yaw_angle_vel_ecd,Yaw_position_pid.output_compensation);

	Yaw_current_pid.Kp=110 ;
	Yaw_current_pid.Ki=60;
	PID_Calc( &Yaw_current_pid, yaw_angle_cur_ecd,Yaw_speed_pid.output_compensation);	
    
	/* ��̨���ݷ��� */
	GimbalDataSend(GimbalSta,Pitch_speed_pid.output,Yaw_current_pid.output_compensation);

	Fdb =	pitch_angle_pos_ecd*100;
	Target = pitch_angle_pos_ref*100;
	
}

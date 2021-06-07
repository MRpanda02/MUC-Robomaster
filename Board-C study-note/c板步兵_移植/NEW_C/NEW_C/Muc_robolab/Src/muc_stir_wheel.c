#include "muc_stir_wheel.h"
#include "muc_pid.h"
#include "muc_remote.h"
#include "muc_shoot_mode.h"
#include "muc_shoot_wheel.h"
#include "tim.h"

		
PidTypeDef Stir_speed_pid={0};
PidTypeDef Stir_position_pid={0};

StirReceiveTypeDef Stir_speed_Feedback;
StirReceiveTypeDef Stir_position_Feedback;

int32_t StirSpeedOut = 0;

void Magazine_Init(void)
{
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
    TIM4->CCR3 =1000;
}

void StirAnalysisKey(void)
{
static int ShootModeChangeFlag=0;	
static int Store_Flag=0;
	if( CC_Condition.Key_B==PressDown&&ShootModeChangeFlag==0)
	{
	ShootModeChangeFlag=1;
	set_st_mode(single);
	}
	
	else if(CC_Condition.Key_G==PressDown&&ShootModeChangeFlag==1)
	{
		ShootModeChangeFlag=0;
	set_st_mode(running);
	}
	

	
	//�����̴���
    if( CC_Condition.MouseLeftBt==PressDown)
    {
        MouseLeftPressTime++;
        Shooting();
    } 
	else
	{
	 MouseLeftPressTime=0;
	}
}
	
void StirAnalysisRemote(void)
{
    if( CC_Condition.S1Sta==StaN2 )
    {
        MouseLeftPressTime++;
        Shooting();
    } 
	else if(CC_Condition.S1Action == SwitchAction_2TO3)
    {		
        MouseLeftPressTime = 0;
    }
}

void StirAnalysisWay(void)
{
	if(g_RcRawData.s2==1){
	StirAnalysisRemote();
	}
	else if(g_RcRawData.s2==3) { 
	StirAnalysisKey();
	}
}


/**
  * ���ֳ�ʼ������  
  */
void MucStirWheelInit( void )
{

	// PID ��ʼ��
//	PID_Init( &Stir_position_pid,2,0,0,0,2200,0,0,28500);
//	PID_Init(&Stir_speed_pid,10,0.1,7, 0,30000,0,10000,7260);
	PID_Init(&Stir_position_pid,2,0,0,0,2200,0,0,28500);
	PID_Init(&Stir_speed_pid,25,0,0,0,4000,0,0,7260);
}


/***************���ֵ�����ٶȷ���ֵ����*************************/
//����̵��������Ϊ����Ƶ�ʲ�һ��
void Shooting_Speed_Data_deal(StirReceiveTypeDef *Receive, uint8_t *msgData)
{
	Receive->real[Receive->count]=(msgData[2]<<8)|msgData[3];//���յ�����ʵ����ֵ
	Receive->count++;
	if(	Receive->count>1)//�Ĵδ���һ��ƽ��ֵ�����շ���Ƶ����1KHZ������ƽ��ֵ�����Ƶ��Ϊ500HZ��PID����Ƶ��Ϊ500HZ
	{
		Receive->calc=(Receive->real[0]+Receive->real[1]);//��������ƽ������ֵ
		Receive->count=0;
	}
}


/*************���ֵ��λ�ô���***************************/
//��Ϊ���ֵ���м��ٱȣ���Ҫ���⴦��һ��
void Position_Round_Data_deal(StirReceiveTypeDef *Receive, uint8_t *msgData )
{

	Receive->real[0]=((msgData[0]<<8)|msgData[1]);//���յ�����ʵ����ֵ
	if((Receive->real[0]-Receive->real[1])>4100)
	{
		Receive->round_cnt--;
	}
	else if((Receive->real[1]-Receive->real[0])>4100)
	{
		Receive->round_cnt++;
	}
	if(Receive->round_cnt>299)
	{
		Receive->round_cnt=0;
	}
	else if(Receive->round_cnt<0)
	{
		Receive->round_cnt=299;
	}
	Receive->ecd_value=(Receive->real[0]/100+82*Receive->round_cnt);
	Receive->real[1]=Receive->real[0];

}


int32_t handledShoot_P_Feedback;
int16_t shooting_position;


extern ShootSpeedSta_t shootStatus;
/**
  * ���ֿ���
  */
 int Shoot_State=0;
 int shootnumber=0;
void SMControlLoop_Task(void)
{
	
	static int time=0;
    static int32_t runTimes=0;
	static int next_runTime=-1;

	time=(time+1)%1000;
	runTimes++;
	runTimes = runTimes%30000;
	static int round=0;
	
/*����Ħ���ֿ����󣬲��ֲſ���ת��*/
	if( shootStatus==Spd_Stop  )
	{
		Shoot_State=HAL_GPIO_ReadPin( GPIOD, GPIO_PIN_15);
		if(Shoot_State==1)	shooting_position-=1;
	}
	else{
		StirAnalysisWay();//����ģʽ
	}
	
// ����ٽ�ֵ����
	if( (Stir_position_Feedback.ecd_value-shooting_position)>20000)
		handledShoot_P_Feedback = Stir_position_Feedback.ecd_value-300*82;
	else if( (shooting_position-Stir_position_Feedback.ecd_value)>20000)
		handledShoot_P_Feedback = Stir_position_Feedback.ecd_value+300*82;
	else
		handledShoot_P_Feedback = Stir_position_Feedback.ecd_value;
	 if(shooting_position>24601)
	 {
		shooting_position-=300*82;
	 }
	 else if(shooting_position<-24601)
	 {
	 shooting_position+=300*82;
	 }
	
	if(runTimes%2==0)
	{
		PID_Calc(&Stir_position_pid,handledShoot_P_Feedback,shooting_position);
		PID_Calc(&Stir_speed_pid,(double)(Stir_speed_Feedback.calc/20),Stir_position_pid.output);
       StirSpeedOut = Stir_speed_pid.output;

	}
}

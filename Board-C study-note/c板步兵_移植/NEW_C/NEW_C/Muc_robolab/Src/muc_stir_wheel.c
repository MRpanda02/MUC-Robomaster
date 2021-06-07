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
	

	
	//鼠标键盘处理
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
  * 拨轮初始化函数  
  */
void MucStirWheelInit( void )
{

	// PID 初始化
//	PID_Init( &Stir_position_pid,2,0,0,0,2200,0,0,28500);
//	PID_Init(&Stir_speed_pid,10,0.1,7, 0,30000,0,10000,7260);
	PID_Init(&Stir_position_pid,2,0,0,0,2200,0,0,28500);
	PID_Init(&Stir_speed_pid,25,0,0,0,4000,0,0,7260);
}


/***************拨轮电机的速度反馈值处理*************************/
//与底盘电机的区别为接受频率不一样
void Shooting_Speed_Data_deal(StirReceiveTypeDef *Receive, uint8_t *msgData)
{
	Receive->real[Receive->count]=(msgData[2]<<8)|msgData[3];//接收到的真实数据值
	Receive->count++;
	if(	Receive->count>1)//四次处理一次平均值，接收反馈频率是1KHZ，数据平均值处理后频率为500HZ，PID处理频率为500HZ
	{
		Receive->calc=(Receive->real[0]+Receive->real[1]);//处理过后的平均数据值
		Receive->count=0;
	}
}


/*************拨轮电机位置处理***************************/
//因为拨轮电机有减速比，需要特殊处理一下
void Position_Round_Data_deal(StirReceiveTypeDef *Receive, uint8_t *msgData )
{

	Receive->real[0]=((msgData[0]<<8)|msgData[1]);//接收到的真实数据值
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
  * 波轮控制
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
	
/*仅当摩擦轮开启后，拨轮才可以转动*/
	if( shootStatus==Spd_Stop  )
	{
		Shoot_State=HAL_GPIO_ReadPin( GPIOD, GPIO_PIN_15);
		if(Shoot_State==1)	shooting_position-=1;
	}
	else{
		StirAnalysisWay();//控制模式
	}
	
// 电机临界值处理
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

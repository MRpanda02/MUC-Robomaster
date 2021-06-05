#include "muc_stir_wheel.h"
#include "muc_remote.h"
#include "muc_shoot_mode.h"
#include "muc_shoot_wheel.h"
#include "tim.h"
#include "muc_hardware_can.h"
#include "muc_main_task.h"
#include "muc_visual_tasks.h"

extern uint32_t mainTaskTimes;
int HavePill = 0;  //当前枪管中有的子弹量
uint8_t Photogate = 1;     //弹管下方光电门      0:有遮挡      1:无遮挡


//底盘反馈信息
int16_t ShootNum_42 = 0;//42mm子弹不超热量允许射击次数
int16_t ShootNum_17 = 0;//17mm子弹不超热量允许射击次数
uint8_t ShootNum_42Flag = 0;
uint8_t Heat42_Have100_flag;//42mm剩余热量 低于100则为1
//波轮控制
void StirAnalysisKey(void)//键盘模式
{	
//鼠标键盘处理
	
	//按G开启小弹模式,SHIFT加G取消	
	static uint8_t Flag_G = 1;//G键按过标志
	static uint8_t Last_Key_G = PressUp;
	
	//G键小弹模式
	if(Flag_G == 0 && CC_Condition.Key_G == PressDown && Last_Key_G == PressUp)
	{
		Flag_G = 1;
	}
	else if(Flag_G == 1 && CC_Condition.Key_SHIFT == PressDown && CC_Condition.Key_G == PressDown)
	{
		Flag_G = 0;
	}
	Last_Key_G = CC_Condition.Key_G;//更新G键状态

	//打弹控制
    if( CC_Condition.MouseLeftBt==PressDown)
    {
			MouseLeftPressTime++;
			if(MouseLeftPressTime == 1)
			{
        if(BigWheelSta == WheelRun)
        {
          if(Photogate == 0)
          {
            SetBigTurn(5);//8//大波轮
          }
          
          //ShootBigPill(1);//大发射机构
          if(Flag_G == 1)
          {
            //SetSmalTurn(8);//小波轮打弹
          }
        }
			}
			else if(MouseLeftPressTime >= 300)
			{
				MouseLeftPressTime = 0;
			}
    }
		else
		{
		 MouseLeftPressTime=0;
		}
		
		if(CC_Condition.MouseRightBt==PressDown)//自瞄模式
		{
			//AutoShootingAndWindmill();
		}
		else//退出自瞄,数据清理
		{
			Version_Shooting.attackCount = 0;
			Version_Shooting.Pitch_orientation = 0;
			Version_Shooting.Yaw_orientation = 0;
		}
}
	
void StirAnalysisRemote(void)//遥控器模式
{
    if(CC_Condition.S1Action == SwitchAction_2TO3)
    {
			if(BigWheelSta == WheelRun)
			{
				if(Photogate == 0)
				{
					SetBigTurn(5);//8//大波轮转一个子弹 
				}
				//ShootBigPill(1);//大发射机构拨一发弹

			}
			if(SmallWheelSta == WheelRun)
			{
				//SetSmalTurn(8);//小波轮
			}
       MouseLeftPressTime = 0;
    }
}

void StirAnalysisWay(void)//控制模式
{
	if(g_RcRawData.s2==1){
	StirAnalysisRemote();
	}
	else if(g_RcRawData.s2==3) { 
	StirAnalysisKey();
	}
}


//大波轮
volatile Encoder BigTurn_Encoder = {0,0,0,0,0,0,0,0,0};//存储can反馈的数据

PidTypeDef Big_Stir_speed_pid={0};
PidTypeDef Big_Stir_position_pid={0};

float BigTurn_Position_ecd = 0;
float BigTurn_Position_ref = 0;

int32_t BigTurn_Speed_ecd = 0;

uint8_t BigRunFlag = 0;//0:停止    1：转    2：旋转中   3：卡弹    4:已到达目标位置附近
uint32_t BigRunTime = 0;//旋转时间

uint8_t BigTurnRunNum = 1;//转动固定角度次数

void SetBigTurn(int Num)//大波轮可转动标志
{
	if(BigRunFlag == 0)
	{ 
		BigRunFlag = 1;
		BigTurnRunNum = Num;
	}
}

void BigTurn_Data_Deal(volatile Encoder *v, uint8_t *msgData)//接收处理大波轮反馈数据
{
	EncoderProcess(v,msgData);
}

void OneTurnSet(void)//大波轮目标角度赋值
{
#define BigOneAngle 50  //设置一次转动的角度
#define BigStayTime 400      //旋转时间限制，超过该值即卡弹

int Wait_time = 0;//BigStayTime;   //转完一个角度后延时
int Wide = 30;//5;                //判定转完一个角度误差
int SpeedFlag = -1;  //+1与-1:改旋转方向
	
static uint16_t PowerOffTime = 0;//断电时间
static uint8_t BigTurnPower = 1;//为0 时断电
  
	StirAnalysisWay();//控制模式
	if(Photogate == 1)//光电门检测
	{ SetBigTurn(1); }
	
	if(BigTurnPower == 0)
	{
		PowerOffTime ++;
	}
	
  if(MyAbs(BigTurn_Position_ecd-BigTurn_Position_ref) > 10*BigOneAngle)
  {
    BigTurn_Position_ref = BigTurn_Position_ecd;
  }

//电机临界值处理
  if(BigTurn_Position_ref>=20000)
  {
    BigTurn_Position_ref -= 50*360;
    BigTurn_Encoder.round_cnt -= 50;
  }
	else if(BigTurn_Position_ref<=-20000)
  {
    BigTurn_Position_ref += 50*360;
    BigTurn_Encoder.round_cnt += 50;
  }
	  
	//转动控制
  if(GetWorkState()==NORMAL_STATE)
  {    
    if(MyAbs((double)BigTurn_Position_ecd-BigTurn_Position_ref) >= 1.2*BigOneAngle && BigRunFlag == 1)
    {
      BigTurn_Position_ref = (double)((int)(BigTurn_Position_ecd/BigOneAngle)*BigOneAngle);
      BigRunFlag = 0;
    }
    if(BigRunFlag == 1)
    {
      BigRunFlag = 2;
      BigRunTime = 0;
      //目标值
      BigTurn_Position_ref += (BigOneAngle)*SpeedFlag*BigTurnRunNum;
    }
    else if(BigRunFlag == 2 && BigRunTime <= BigStayTime && MyAbs((double)BigTurn_Position_ecd-BigTurn_Position_ref) <= Wide)
    {
      BigRunFlag = 4;
    }
    else if(BigRunFlag == 2 && BigRunTime > BigStayTime) //卡弹
    {
//      BigTurn_Position_ref -= (7*BigOneAngle)*SpeedFlag;
			BigTurnPower = 0;//卡弹断电
      BigRunFlag = 3;
      BigRunTime = 0;
    }
    else if(BigRunFlag == 4)
    {//已达到目标位置附近,延时处理
      BigRunTime++;
      if(BigRunTime > Wait_time)//BigStayTime
			{ BigRunFlag = 0; }
    }
    else
      BigRunTime++;
    
    //if((BigRunFlag == 3 && MyAbs((double)BigTurn_Position_ecd-BigTurn_Position_ref) < 5))//200
    if(BigRunFlag == 3 && PowerOffTime >= 400)
		{
				BigTurnPower  = 1;
				PowerOffTime = 0;
				BigRunFlag = 0;
    }
  }
	
		BigTurn_Position_ecd = BigTurn_Encoder.ecd_angle;
		BigTurn_Speed_ecd = BigTurn_Encoder.filter_rate;
		
		PID_Calc(&Big_Stir_position_pid,BigTurn_Position_ecd,BigTurn_Position_ref);
		PID_Calc(&Big_Stir_speed_pid,BigTurn_Speed_ecd,Big_Stir_position_pid.output);
	  Big_Stir_speed_pid.output *= BigTurnPower;

}





//大发射机构推弹:新英雄大发射机构推弹不是用小波轮
volatile Encoder NewTurn_Encoder = {0,0,0,0,0,0,0,0,0};//存储can反馈的数据
PidTypeDef NewTurn_Speed_pid = {0};
PidTypeDef NewTurn_Position_pid = {0};
float NewTurn_Position_ecd = 0;
int32_t NewTurn_Speed_ecd = 0;
uint32_t BigPauseTime = 0;//大弹每两发间隔时间,防止超速
int ShootNum = 0;//往返次数


void NewTurn_Data_Deal(volatile Encoder *v, uint8_t *msgData)//接收处理新推弹机构反馈数据
{
	EncoderProcess(v,msgData);
}

void NewTurnInit()
{
//	PID_Init( &NewTurn_Position_pid,0.2,0,0,0,2200,0,0,28500); //Kp=0.12
//	PID_Init( &NewTurn_Speed_pid,100,0,0,0,8000,0,0,7260);//kp55
	
		PID_Init( &NewTurn_Position_pid,0.02,0,0,0,2200,0,0,28500);//0.03
	PID_Init( &NewTurn_Speed_pid,80,0,0,0,9000,0,0,7260);//80

}

#define oneBigShootLifeTime 1700 //ms
uint32_t RestTime = oneBigShootLifeTime;
void ShootBigPill()//设置拨弹次数
{
  if(Heat42_Have100_flag == 1)
  {
    RestTime = oneBigShootLifeTime;
  }
  else
  {
    RestTime = 300;
  }
		//发射数量限制: 满足  小于等于最大允许数&滑块已复位&与上次发射间隔时间大于300ms 才可发射 
		if(BigPauseTime >= RestTime)
		{
			if(ShootNum_42 >= 1 && ShootNum == 0 && ShootNum_42Flag == 1)
			{
				ShootNum_42Flag = 0;
				BigPauseTime = 0;
				ShootNum_42--;
				ShootNum = 1; // 立推杆标志
			}
		}
    else
    {
      ShootNum = 0;
    }
}

int NewTurnMoveFlag = -1;
float BackRunk = -2500;//起始位置对应的电机位置
uint8_t Gate = 0;//枪管后部微动开关 状态: 1为未按   0为按下
uint32_t WaitTime = 0;//运动时间

void NewTurnTask(int Long)//拨弹控制函数: Long:前后电机位置差  Speed:转动速度,方向为从起始位置到最远处,注意正负
{
#define StopRunTime 500 //检测卡弹时长
	int SpeedFlag = 1;//电机速度方向 +1 -1改方向
  
	BigPauseTime ++;
	if(BigPauseTime >= 400) //每次向前运动400ms
	{
		ShootNum = 0;
		NewTurnMoveFlag = -1;//向后运动
	}
	//卡弹处理
	if((WaitTime >= StopRunTime || (MyAbs(NewTurn_Encoder.filter_rate)<30 && WaitTime > 300))&&NewTurnMoveFlag == 1) //认为卡弹
	{
		ShootNum = 0;//卡弹清除子弹数,防止连射超热量
		NewTurnMoveFlag = -1;
		WaitTime = 0;
	}
	//读取限位开关
	Gate = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	
	WaitTime ++;
	if(BigWheelSta == WheelRun)//允许发弹
	{
    if(ShootNum == 1)
    {
			if(NewTurnMoveFlag == 0)
			{
				NewTurnMoveFlag = 1;
				WaitTime = 0;
			}
    }
    else
    {
      NewTurnMoveFlag = -1;
    }
	}
	else//不打弹
	{
		BigPauseTime = oneBigShootLifeTime;
    NewTurnMoveFlag = -1;
		ShootNum = 0;
	}
	
	if((Gate == 0 && WaitTime > 100) || (MyAbs(NewTurn_Encoder.filter_rate)<30 && WaitTime >= StopRunTime && NewTurnMoveFlag == -1))//压下状态
	{
		NewTurnMoveFlag = 0;
		BackRunk = NewTurn_Encoder.ecd_angle;//返回后校准一次位置
	}

	//临界值处理
	if(BackRunk>=20000)
  {
    BackRunk -= 50*360;
    NewTurn_Encoder.round_cnt -= 50;
  }
	else if(BackRunk<=-20000)
  {
    BackRunk += 50*360;
    NewTurn_Encoder.round_cnt += 50;
  }

  if(ShootNum_42Flag == 1 && ShootNum_42 <= 0)
    NewTurnMoveFlag = 0;
//速度计算赋值
	PID_Calc(&NewTurn_Position_pid,NewTurn_Encoder.ecd_angle,(BackRunk + SpeedFlag*Long*NewTurnMoveFlag)); 

	PID_Calc(&NewTurn_Speed_pid,NewTurn_Encoder.filter_rate/10,NewTurn_Position_pid.output);
	
//电流发送
	CAN_Send_Message(&hcan1,0x200,Shoot_left_speed_pid.output, Shoot_right_speed_pid.output,Big_Stir_speed_pid.output,0);
	//if(mainTaskTimes % 2 == 0)
	//{
	//	CAN_Send_Message(&hcan2,0x200,0,0,0,Big_Stir_speed_pid.output);
	//}
}

//小波轮
volatile Encoder SmallTurn_Encoder = {0,0,0,0,0,0,0,0,0};//存储can反馈的数据

PidTypeDef Sma_Stir_speed_pid={0};
PidTypeDef Sma_Stir_position_pid={0};

float SmaTurn_Position_ecd = 0;
float SmaTurn_Position_ref = 0;
int32_t SmaTurn_Speed_ecd = 0;

uint32_t SmaRunTime = 0;//旋转时间
uint8_t SmaRunFlag = 0;//0:停止    1：转    2：旋转中   3：卡弹   4:已到达目标位置附近
uint8_t SmallInitFlag = 1;//小弹初始化标志:初始化时,慢速旋转,弹管中加满子弹,直到打出一发子弹

uint8_t SmallTurnRunNum = 1;//小波轮转动固定角度次数

void SetSmalTurn(int Num)//小波轮可转动标志
{
	if(SmaRunFlag == 0)
	{
		SmaRunFlag = 1;
	}
	if(SmallTurnRunNum == 0)
	{ SmallTurnRunNum = Num; }
	if(SmallTurnRunNum >= ShootNum_17)
	{
		SmallTurnRunNum = ShootNum_17;
	}
}


void SmaTurn_Data_Deal(volatile Encoder *v, uint8_t *msgData)//接收处理大波轮反馈数据
{
	EncoderProcess(v,msgData);
}

void SmallTurn_Task()//小波轮控制         
{
#define SmaOneAngle 930.0f //一个子弹角度
#define SmaStayTime 300//400 //卡弹判定时间
	
	int SpeedFlag = 1;//+1 -1控制转动方向
	if(SmallInitFlag == 1 && SmallWheelSta == WheelRun)//开局时填满弹管
	{
		SetSmalTurn(1);
	}
  
  if(GetWorkState()==NORMAL_STATE)  //小波轮
  {
    static uint32_t SmaRunTime = 0;//移动时间记录
    static float StayFlag = 0; //记录卡弹后回退目标位置
    
    if(GMPitchEncoder.ecd_angle <= -2.5)
    {
      SmaRunFlag = 0;
      SmallTurnRunNum = 0;
      SmaTurn_Position_ref = SmallTurn_Encoder.ecd_angle;
    }
    
    if(SmaRunFlag == 1 && SmaTurn_Position_ref-SmallTurn_Encoder.ecd_angle < 0.8 * SmaOneAngle)//可转动
    {
      SmaRunFlag = 2;
      SmaRunTime = 0;
      //目标值
      SmaTurn_Position_ref += SmaOneAngle * SpeedFlag * SmallTurnRunNum;
      StayFlag = SmaTurn_Position_ref-SmaOneAngle * SpeedFlag * SmallTurnRunNum;
    }
    //else if(SmaRunFlag == 2 && SmaRunTime <= SmaStayTime *(uint16_t)(MyAbs(SmaTurn_Position_ref - SmallTurn_Encoder.ecd_angle) / (SmaOneAngle - 100)) && MyAbs((double)(SmallTurn_Encoder.ecd_angle-SmaTurn_Position_ref)) < 0.8 * SmaOneAngle)
    else if(SmaRunFlag == 2 && SmaRunTime <= SmaStayTime * SmallTurnRunNum && MyAbs((double)(SmallTurn_Encoder.ecd_angle-SmaTurn_Position_ref)) < 0.8 * SmaOneAngle)//转动中
		{
      SmaRunFlag = 4;
    }
    else if(SmaRunFlag == 2 && SmaRunTime > SmaStayTime * SmallTurnRunNum) //卡弹
    {
      SmaTurn_Position_ref = SmallTurn_Encoder.ecd_angle - SmaOneAngle * SpeedFlag * 3;
      SmaRunFlag = 3;
			SmaRunTime = 0;
    }
    else if(SmaRunFlag == 4)//转动完成
    {
      SmaRunTime++;
//      if(SmaRunTime > SmaStayTime * SmallTurnRunNum) //延时处理
//			{
				SmaRunFlag = 0;
				SmallTurnRunNum = 0;
//			}
    }
    else
      SmaRunTime++;
    
    if(SmaRunFlag == 3)//卡弹状态:倒退,若倒退时卡弹,停止倒退
    {
      if(MyAbs((double)(SmallTurn_Encoder.ecd_angle-SmaTurn_Position_ref)) < 0.8 * SmaOneAngle)
				SmaRunFlag = 5;
			else if(SmaRunTime >= 1000)
				SmaRunFlag = 5;
				
    }
		else if(SmaRunFlag == 5)//卡弹回转后继续
    {
      SmaTurn_Position_ref = SmallTurn_Encoder.ecd_angle;
      SmaRunFlag = 2;
      SmaRunTime = 0;
			SmallTurnRunNum = 0;
    }
	}
	else
	{
		SmaRunTime = 0;
		SmallTurnRunNum = 0;
		SmaRunFlag = 0;
		SmaTurn_Position_ref = SmallTurn_Encoder.ecd_angle;
	}
	
	
	if(MyAbs(SmallTurn_Encoder.ecd_angle-SmaTurn_Position_ref) > (5+SmallTurnRunNum)*SmaOneAngle)
  {
    SmaTurn_Position_ref = SmallTurn_Encoder.ecd_angle;
  }
	//临界值处理
  if(SmaTurn_Position_ref >= 20000)
  {
    SmaTurn_Position_ref -= 50*360;
    SmallTurn_Encoder.round_cnt -= 50;
  }
	else
	if(SmaTurn_Position_ref <= -20000)
  {
    SmaTurn_Position_ref += 50*360;
    SmallTurn_Encoder.round_cnt += 50;
  }

	SmaTurn_Position_ecd = SmallTurn_Encoder.ecd_angle;
	SmaTurn_Speed_ecd = SmallTurn_Encoder.filter_rate/10;
	
	//SmaTurn_Position_ref = 0;
	PID_Calc(&Sma_Stir_position_pid,SmaTurn_Position_ecd,SmaTurn_Position_ref);
	PID_Calc(&Sma_Stir_speed_pid,SmaTurn_Speed_ecd,Sma_Stir_position_pid.output);

	if(MyAbs(SmallTurn_Encoder.filter_rate) >= 750)//疯转处理
	{
		Sma_Stir_speed_pid.output = 0;
		SmaTurn_Position_ref = SmaTurn_Position_ecd;
		SmaRunFlag = 0;
		SmallTurnRunNum = 0;
	}
	
	//CAN_Send_Message( &hcan2, 0X1ff, 0, 0, Sma_Stir_speed_pid.output, 0 );
	CAN_Send_Message( &hcan2, 0X1ff, 0, Yaw_speed_pid.output, 0, 0 );
}





//弹仓盖 (电机2006)
CMReceiveTypeDef MagazinePossionFeedback;   //位置反馈信息
CMReceiveTypeDef MagazineSpeedFeedback;   //速度反馈信息

PidTypeDef Magazine_possion_pid={0};//位置环PID
PidTypeDef Magazine_speed_pid={0};//速度环PID

int16_t MagazinePossion_ref=0;//目标位置
int16_t MagazinePossion_ecd=0;//实际位置
int16_t MagazineSpeed_ecd=0;//实际速度

void Magazine_Init(void)
{
	PID_Init(&Magazine_possion_pid,1.2,0,0,0,2200,0,0,28500);
	PID_Init(&Magazine_speed_pid,55,0,0,0,5000,0,0,7260);
}
void MagazinePossionDataDeal(CMReceiveTypeDef *Receive,uint8_t * msgData)//弹仓盖反馈位置处理
{
	Receive->real[0]=((msgData[0]<<8)|msgData[1]);//接收到的真实数据值
	//电机减速比处理
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
  
	//实际位置处理
	if( (MagazinePossionFeedback.ecd_value-MagazinePossion_ref)>20000)
		MagazinePossion_ecd = MagazinePossionFeedback.ecd_value-300*82;
	else if( (MagazinePossion_ref-MagazinePossionFeedback.ecd_value)>20000)
		MagazinePossion_ecd = MagazinePossionFeedback.ecd_value+300*82;
	else
		MagazinePossion_ecd = MagazinePossionFeedback.ecd_value;
	 if(MagazinePossion_ref>24601)
		MagazinePossion_ref-=300*82;
}

void MagazineSpeedDataDeal(CMReceiveTypeDef *Receive,uint8_t * msgData)//弹仓盖反馈速度处理
{
 Receive->real[Receive->count]=(msgData[2]<<8)|msgData[3];
	Receive->count++;
	if(	Receive->count>1)
	{
		Receive->calc=(Receive->real[0]+Receive->real[1])/2;
		MagazineSpeed_ecd=Receive->calc/20;
		Receive->count=0;
	}
}



/**
  * 拨轮初始化函数  
  */
void MucStirWheelInit( void ) // PID 初始化
{
  //小发射机构波轮
	PID_Init( &Sma_Stir_position_pid,0.015,0,0,0,2200,0,0,28500);//0.03
	PID_Init( &Sma_Stir_speed_pid,80,0,0,0,9000,0,0,7260);//80

	//大弹大波轮
	PID_Init( &Big_Stir_position_pid,2,0,0,0,2200,0,0,28500);//kp 3
  PID_Init( &Big_Stir_speed_pid,100,0,0,0,10000,0,0,7260);//kp120
	
	//大弹推弹初始化
	NewTurnInit();
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



extern ShootSpeedSta_t shootStatus;
/**
  * 波轮控制
  */
void SMControlLoop_Task(void)
{	
//	static int Photogate = 1;     //开关状态      0:有遮挡      1:无遮挡
  static int PillFlag = 0;     //子弹通过标志位,为1表示有子弹遮挡光电门
  static uint16_t CoverTime = 0;//遮挡时间 
  static int LastPhotogate = 0; //上一次光电门状态

	Photogate=0;//HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);//获取当前光电门状态
	
	//大波轮
		//OneTurnSet();
	
	//大弹推弹机构
	//NewTurnTask(3500);

	//小波轮
	//SmallTurn_Task();
	
	//LastPhotogate = Photogate;//光电门状态更新
}

int16_t HaveHeat42 = 0;
void ShootDataDeal( uint8_t * msgData )//处理底盘返回的射击信息
{
	ShootNum_17 = (int16_t)(msgData[2]<<8 | msgData[3]);
	if((int16_t)(msgData[4]<<8 | msgData[5]) == 1)//42mm已发射,发射间隔时间置零等待
	{
    if(ShootNum_42Flag == 0)
    {
      ShootNum_42Flag = 1;
    }
    HaveHeat42 = (int16_t)(msgData[6]<<8 | msgData[7]);
    if(HaveHeat42 <= 200)
      Heat42_Have100_flag = 1;
    else
      Heat42_Have100_flag = 0;
    
    ShootNum_42 = ((int16_t)(msgData[0]<<8 | msgData[1])>=1)?1:0;
	}
}

void SensorInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitStruct.Mode = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}








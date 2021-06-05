#include "muc_stir_wheel.h"
#include "muc_remote.h"
#include "muc_shoot_mode.h"
#include "muc_shoot_wheel.h"
#include "tim.h"
#include "muc_hardware_can.h"
#include "muc_main_task.h"
#include "muc_visual_tasks.h"

extern uint32_t mainTaskTimes;
int HavePill = 0;  //��ǰǹ�����е��ӵ���
uint8_t Photogate = 1;     //�����·������      0:���ڵ�      1:���ڵ�


//���̷�����Ϣ
int16_t ShootNum_42 = 0;//42mm�ӵ��������������������
int16_t ShootNum_17 = 0;//17mm�ӵ��������������������
uint8_t ShootNum_42Flag = 0;
uint8_t Heat42_Have100_flag;//42mmʣ������ ����100��Ϊ1
//���ֿ���
void StirAnalysisKey(void)//����ģʽ
{	
//�����̴���
	
	//��G����С��ģʽ,SHIFT��Gȡ��	
	static uint8_t Flag_G = 1;//G��������־
	static uint8_t Last_Key_G = PressUp;
	
	//G��С��ģʽ
	if(Flag_G == 0 && CC_Condition.Key_G == PressDown && Last_Key_G == PressUp)
	{
		Flag_G = 1;
	}
	else if(Flag_G == 1 && CC_Condition.Key_SHIFT == PressDown && CC_Condition.Key_G == PressDown)
	{
		Flag_G = 0;
	}
	Last_Key_G = CC_Condition.Key_G;//����G��״̬

	//�򵯿���
    if( CC_Condition.MouseLeftBt==PressDown)
    {
			MouseLeftPressTime++;
			if(MouseLeftPressTime == 1)
			{
        if(BigWheelSta == WheelRun)
        {
          if(Photogate == 0)
          {
            SetBigTurn(5);//8//����
          }
          
          //ShootBigPill(1);//�������
          if(Flag_G == 1)
          {
            //SetSmalTurn(8);//С���ִ�
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
		
		if(CC_Condition.MouseRightBt==PressDown)//����ģʽ
		{
			//AutoShootingAndWindmill();
		}
		else//�˳�����,��������
		{
			Version_Shooting.attackCount = 0;
			Version_Shooting.Pitch_orientation = 0;
			Version_Shooting.Yaw_orientation = 0;
		}
}
	
void StirAnalysisRemote(void)//ң����ģʽ
{
    if(CC_Condition.S1Action == SwitchAction_2TO3)
    {
			if(BigWheelSta == WheelRun)
			{
				if(Photogate == 0)
				{
					SetBigTurn(5);//8//����תһ���ӵ� 
				}
				//ShootBigPill(1);//���������һ����

			}
			if(SmallWheelSta == WheelRun)
			{
				//SetSmalTurn(8);//С����
			}
       MouseLeftPressTime = 0;
    }
}

void StirAnalysisWay(void)//����ģʽ
{
	if(g_RcRawData.s2==1){
	StirAnalysisRemote();
	}
	else if(g_RcRawData.s2==3) { 
	StirAnalysisKey();
	}
}


//����
volatile Encoder BigTurn_Encoder = {0,0,0,0,0,0,0,0,0};//�洢can����������

PidTypeDef Big_Stir_speed_pid={0};
PidTypeDef Big_Stir_position_pid={0};

float BigTurn_Position_ecd = 0;
float BigTurn_Position_ref = 0;

int32_t BigTurn_Speed_ecd = 0;

uint8_t BigRunFlag = 0;//0:ֹͣ    1��ת    2����ת��   3������    4:�ѵ���Ŀ��λ�ø���
uint32_t BigRunTime = 0;//��תʱ��

uint8_t BigTurnRunNum = 1;//ת���̶��Ƕȴ���

void SetBigTurn(int Num)//���ֿ�ת����־
{
	if(BigRunFlag == 0)
	{ 
		BigRunFlag = 1;
		BigTurnRunNum = Num;
	}
}

void BigTurn_Data_Deal(volatile Encoder *v, uint8_t *msgData)//���մ�����ַ�������
{
	EncoderProcess(v,msgData);
}

void OneTurnSet(void)//����Ŀ��Ƕȸ�ֵ
{
#define BigOneAngle 50  //����һ��ת���ĽǶ�
#define BigStayTime 400      //��תʱ�����ƣ�������ֵ������

int Wait_time = 0;//BigStayTime;   //ת��һ���ǶȺ���ʱ
int Wide = 30;//5;                //�ж�ת��һ���Ƕ����
int SpeedFlag = -1;  //+1��-1:����ת����
	
static uint16_t PowerOffTime = 0;//�ϵ�ʱ��
static uint8_t BigTurnPower = 1;//Ϊ0 ʱ�ϵ�
  
	StirAnalysisWay();//����ģʽ
	if(Photogate == 1)//����ż��
	{ SetBigTurn(1); }
	
	if(BigTurnPower == 0)
	{
		PowerOffTime ++;
	}
	
  if(MyAbs(BigTurn_Position_ecd-BigTurn_Position_ref) > 10*BigOneAngle)
  {
    BigTurn_Position_ref = BigTurn_Position_ecd;
  }

//����ٽ�ֵ����
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
	  
	//ת������
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
      //Ŀ��ֵ
      BigTurn_Position_ref += (BigOneAngle)*SpeedFlag*BigTurnRunNum;
    }
    else if(BigRunFlag == 2 && BigRunTime <= BigStayTime && MyAbs((double)BigTurn_Position_ecd-BigTurn_Position_ref) <= Wide)
    {
      BigRunFlag = 4;
    }
    else if(BigRunFlag == 2 && BigRunTime > BigStayTime) //����
    {
//      BigTurn_Position_ref -= (7*BigOneAngle)*SpeedFlag;
			BigTurnPower = 0;//�����ϵ�
      BigRunFlag = 3;
      BigRunTime = 0;
    }
    else if(BigRunFlag == 4)
    {//�ѴﵽĿ��λ�ø���,��ʱ����
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





//��������Ƶ�:��Ӣ�۴�������Ƶ�������С����
volatile Encoder NewTurn_Encoder = {0,0,0,0,0,0,0,0,0};//�洢can����������
PidTypeDef NewTurn_Speed_pid = {0};
PidTypeDef NewTurn_Position_pid = {0};
float NewTurn_Position_ecd = 0;
int32_t NewTurn_Speed_ecd = 0;
uint32_t BigPauseTime = 0;//��ÿ�������ʱ��,��ֹ����
int ShootNum = 0;//��������


void NewTurn_Data_Deal(volatile Encoder *v, uint8_t *msgData)//���մ������Ƶ�������������
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
void ShootBigPill()//���ò�������
{
  if(Heat42_Have100_flag == 1)
  {
    RestTime = oneBigShootLifeTime;
  }
  else
  {
    RestTime = 300;
  }
		//������������: ����  С�ڵ������������&�����Ѹ�λ&���ϴη�����ʱ�����300ms �ſɷ��� 
		if(BigPauseTime >= RestTime)
		{
			if(ShootNum_42 >= 1 && ShootNum == 0 && ShootNum_42Flag == 1)
			{
				ShootNum_42Flag = 0;
				BigPauseTime = 0;
				ShootNum_42--;
				ShootNum = 1; // ���Ƹ˱�־
			}
		}
    else
    {
      ShootNum = 0;
    }
}

int NewTurnMoveFlag = -1;
float BackRunk = -2500;//��ʼλ�ö�Ӧ�ĵ��λ��
uint8_t Gate = 0;//ǹ�ܺ�΢������ ״̬: 1Ϊδ��   0Ϊ����
uint32_t WaitTime = 0;//�˶�ʱ��

void NewTurnTask(int Long)//�������ƺ���: Long:ǰ����λ�ò�  Speed:ת���ٶ�,����Ϊ����ʼλ�õ���Զ��,ע������
{
#define StopRunTime 500 //��⿨��ʱ��
	int SpeedFlag = 1;//����ٶȷ��� +1 -1�ķ���
  
	BigPauseTime ++;
	if(BigPauseTime >= 400) //ÿ����ǰ�˶�400ms
	{
		ShootNum = 0;
		NewTurnMoveFlag = -1;//����˶�
	}
	//��������
	if((WaitTime >= StopRunTime || (MyAbs(NewTurn_Encoder.filter_rate)<30 && WaitTime > 300))&&NewTurnMoveFlag == 1) //��Ϊ����
	{
		ShootNum = 0;//��������ӵ���,��ֹ���䳬����
		NewTurnMoveFlag = -1;
		WaitTime = 0;
	}
	//��ȡ��λ����
	Gate = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	
	WaitTime ++;
	if(BigWheelSta == WheelRun)//������
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
	else//����
	{
		BigPauseTime = oneBigShootLifeTime;
    NewTurnMoveFlag = -1;
		ShootNum = 0;
	}
	
	if((Gate == 0 && WaitTime > 100) || (MyAbs(NewTurn_Encoder.filter_rate)<30 && WaitTime >= StopRunTime && NewTurnMoveFlag == -1))//ѹ��״̬
	{
		NewTurnMoveFlag = 0;
		BackRunk = NewTurn_Encoder.ecd_angle;//���غ�У׼һ��λ��
	}

	//�ٽ�ֵ����
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
//�ٶȼ��㸳ֵ
	PID_Calc(&NewTurn_Position_pid,NewTurn_Encoder.ecd_angle,(BackRunk + SpeedFlag*Long*NewTurnMoveFlag)); 

	PID_Calc(&NewTurn_Speed_pid,NewTurn_Encoder.filter_rate/10,NewTurn_Position_pid.output);
	
//��������
	CAN_Send_Message(&hcan1,0x200,Shoot_left_speed_pid.output, Shoot_right_speed_pid.output,Big_Stir_speed_pid.output,0);
	//if(mainTaskTimes % 2 == 0)
	//{
	//	CAN_Send_Message(&hcan2,0x200,0,0,0,Big_Stir_speed_pid.output);
	//}
}

//С����
volatile Encoder SmallTurn_Encoder = {0,0,0,0,0,0,0,0,0};//�洢can����������

PidTypeDef Sma_Stir_speed_pid={0};
PidTypeDef Sma_Stir_position_pid={0};

float SmaTurn_Position_ecd = 0;
float SmaTurn_Position_ref = 0;
int32_t SmaTurn_Speed_ecd = 0;

uint32_t SmaRunTime = 0;//��תʱ��
uint8_t SmaRunFlag = 0;//0:ֹͣ    1��ת    2����ת��   3������   4:�ѵ���Ŀ��λ�ø���
uint8_t SmallInitFlag = 1;//С����ʼ����־:��ʼ��ʱ,������ת,�����м����ӵ�,ֱ�����һ���ӵ�

uint8_t SmallTurnRunNum = 1;//С����ת���̶��Ƕȴ���

void SetSmalTurn(int Num)//С���ֿ�ת����־
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


void SmaTurn_Data_Deal(volatile Encoder *v, uint8_t *msgData)//���մ�����ַ�������
{
	EncoderProcess(v,msgData);
}

void SmallTurn_Task()//С���ֿ���         
{
#define SmaOneAngle 930.0f //һ���ӵ��Ƕ�
#define SmaStayTime 300//400 //�����ж�ʱ��
	
	int SpeedFlag = 1;//+1 -1����ת������
	if(SmallInitFlag == 1 && SmallWheelSta == WheelRun)//����ʱ��������
	{
		SetSmalTurn(1);
	}
  
  if(GetWorkState()==NORMAL_STATE)  //С����
  {
    static uint32_t SmaRunTime = 0;//�ƶ�ʱ���¼
    static float StayFlag = 0; //��¼���������Ŀ��λ��
    
    if(GMPitchEncoder.ecd_angle <= -2.5)
    {
      SmaRunFlag = 0;
      SmallTurnRunNum = 0;
      SmaTurn_Position_ref = SmallTurn_Encoder.ecd_angle;
    }
    
    if(SmaRunFlag == 1 && SmaTurn_Position_ref-SmallTurn_Encoder.ecd_angle < 0.8 * SmaOneAngle)//��ת��
    {
      SmaRunFlag = 2;
      SmaRunTime = 0;
      //Ŀ��ֵ
      SmaTurn_Position_ref += SmaOneAngle * SpeedFlag * SmallTurnRunNum;
      StayFlag = SmaTurn_Position_ref-SmaOneAngle * SpeedFlag * SmallTurnRunNum;
    }
    //else if(SmaRunFlag == 2 && SmaRunTime <= SmaStayTime *(uint16_t)(MyAbs(SmaTurn_Position_ref - SmallTurn_Encoder.ecd_angle) / (SmaOneAngle - 100)) && MyAbs((double)(SmallTurn_Encoder.ecd_angle-SmaTurn_Position_ref)) < 0.8 * SmaOneAngle)
    else if(SmaRunFlag == 2 && SmaRunTime <= SmaStayTime * SmallTurnRunNum && MyAbs((double)(SmallTurn_Encoder.ecd_angle-SmaTurn_Position_ref)) < 0.8 * SmaOneAngle)//ת����
		{
      SmaRunFlag = 4;
    }
    else if(SmaRunFlag == 2 && SmaRunTime > SmaStayTime * SmallTurnRunNum) //����
    {
      SmaTurn_Position_ref = SmallTurn_Encoder.ecd_angle - SmaOneAngle * SpeedFlag * 3;
      SmaRunFlag = 3;
			SmaRunTime = 0;
    }
    else if(SmaRunFlag == 4)//ת�����
    {
      SmaRunTime++;
//      if(SmaRunTime > SmaStayTime * SmallTurnRunNum) //��ʱ����
//			{
				SmaRunFlag = 0;
				SmallTurnRunNum = 0;
//			}
    }
    else
      SmaRunTime++;
    
    if(SmaRunFlag == 3)//����״̬:����,������ʱ����,ֹͣ����
    {
      if(MyAbs((double)(SmallTurn_Encoder.ecd_angle-SmaTurn_Position_ref)) < 0.8 * SmaOneAngle)
				SmaRunFlag = 5;
			else if(SmaRunTime >= 1000)
				SmaRunFlag = 5;
				
    }
		else if(SmaRunFlag == 5)//������ת�����
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
	//�ٽ�ֵ����
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

	if(MyAbs(SmallTurn_Encoder.filter_rate) >= 750)//��ת����
	{
		Sma_Stir_speed_pid.output = 0;
		SmaTurn_Position_ref = SmaTurn_Position_ecd;
		SmaRunFlag = 0;
		SmallTurnRunNum = 0;
	}
	
	//CAN_Send_Message( &hcan2, 0X1ff, 0, 0, Sma_Stir_speed_pid.output, 0 );
	CAN_Send_Message( &hcan2, 0X1ff, 0, Yaw_speed_pid.output, 0, 0 );
}





//���ָ� (���2006)
CMReceiveTypeDef MagazinePossionFeedback;   //λ�÷�����Ϣ
CMReceiveTypeDef MagazineSpeedFeedback;   //�ٶȷ�����Ϣ

PidTypeDef Magazine_possion_pid={0};//λ�û�PID
PidTypeDef Magazine_speed_pid={0};//�ٶȻ�PID

int16_t MagazinePossion_ref=0;//Ŀ��λ��
int16_t MagazinePossion_ecd=0;//ʵ��λ��
int16_t MagazineSpeed_ecd=0;//ʵ���ٶ�

void Magazine_Init(void)
{
	PID_Init(&Magazine_possion_pid,1.2,0,0,0,2200,0,0,28500);
	PID_Init(&Magazine_speed_pid,55,0,0,0,5000,0,0,7260);
}
void MagazinePossionDataDeal(CMReceiveTypeDef *Receive,uint8_t * msgData)//���ָǷ���λ�ô���
{
	Receive->real[0]=((msgData[0]<<8)|msgData[1]);//���յ�����ʵ����ֵ
	//������ٱȴ���
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
  
	//ʵ��λ�ô���
	if( (MagazinePossionFeedback.ecd_value-MagazinePossion_ref)>20000)
		MagazinePossion_ecd = MagazinePossionFeedback.ecd_value-300*82;
	else if( (MagazinePossion_ref-MagazinePossionFeedback.ecd_value)>20000)
		MagazinePossion_ecd = MagazinePossionFeedback.ecd_value+300*82;
	else
		MagazinePossion_ecd = MagazinePossionFeedback.ecd_value;
	 if(MagazinePossion_ref>24601)
		MagazinePossion_ref-=300*82;
}

void MagazineSpeedDataDeal(CMReceiveTypeDef *Receive,uint8_t * msgData)//���ָǷ����ٶȴ���
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
  * ���ֳ�ʼ������  
  */
void MucStirWheelInit( void ) // PID ��ʼ��
{
  //С�����������
	PID_Init( &Sma_Stir_position_pid,0.015,0,0,0,2200,0,0,28500);//0.03
	PID_Init( &Sma_Stir_speed_pid,80,0,0,0,9000,0,0,7260);//80

	//�󵯴���
	PID_Init( &Big_Stir_position_pid,2,0,0,0,2200,0,0,28500);//kp 3
  PID_Init( &Big_Stir_speed_pid,100,0,0,0,10000,0,0,7260);//kp120
	
	//���Ƶ���ʼ��
	NewTurnInit();
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



extern ShootSpeedSta_t shootStatus;
/**
  * ���ֿ���
  */
void SMControlLoop_Task(void)
{	
//	static int Photogate = 1;     //����״̬      0:���ڵ�      1:���ڵ�
  static int PillFlag = 0;     //�ӵ�ͨ����־λ,Ϊ1��ʾ���ӵ��ڵ������
  static uint16_t CoverTime = 0;//�ڵ�ʱ�� 
  static int LastPhotogate = 0; //��һ�ι����״̬

	Photogate=0;//HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1);//��ȡ��ǰ�����״̬
	
	//����
		//OneTurnSet();
	
	//���Ƶ�����
	//NewTurnTask(3500);

	//С����
	//SmallTurn_Task();
	
	//LastPhotogate = Photogate;//�����״̬����
}

int16_t HaveHeat42 = 0;
void ShootDataDeal( uint8_t * msgData )//������̷��ص������Ϣ
{
	ShootNum_17 = (int16_t)(msgData[2]<<8 | msgData[3]);
	if((int16_t)(msgData[4]<<8 | msgData[5]) == 1)//42mm�ѷ���,������ʱ������ȴ�
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








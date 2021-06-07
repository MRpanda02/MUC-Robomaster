#include "muc_Shoot_Judge_Control.h"
#include "muc_hardware_can.h"
#include "muc_shoot_wheel.h"
#include "muc_referee_system.h"

#define LEVEVL1_HEAT 230//240
#define LEVEVL2_HEAT 350//360
#define LEVEVL3_HEAT 470//480

uint16_t Shoot_Heat=0;      //17mm弹丸热量
uint16_t Shoot_Last_Heat;   //上一次17mm弹丸热量
uint8_t  Shoot_Speed;       //射速 (自己估计的最大值)
uint16_t Shoot_Max_Heat=230;//当前热量上限
uint8_t  Robot_Level;       //步兵等级
uint8_t  Heat_Limit=0;      //热量限制射击标志
int  Remain_Shoot_Number=0; //剩余可发射子弹数

//ReceiveData judge_trans_mesg={0};

receive_from_chassis shoot_control_consulation={0};

void Deal_Data_From_Chassis1( receive_from_chassis  *FromC, uint8_t * msgData)
{
	static int HeatTime=0;
	HeatTime=(HeatTime+1)%100;
	
	FromC->shoot_heat =(msgData[0]<<8)|msgData[1];
	FromC->robot_level=(msgData[2]<<8)|msgData[3];
	FromC->remain_hp  =(msgData[4]<<8)|msgData[5];
	FromC->max_hp	  =(msgData[6]<<8)|msgData[7];

	Shoot_Heat =shoot_control_consulation.shoot_heat;//当前枪口热量
	
	if(HeatTime==0)
	{
		
	Shoot_Heat=0;
	
	}
	Remain_Shoot_Number=( (int)Shoot_Max_Heat-(int)Shoot_Heat ) / Shoot_Speed;
	if(Remain_Shoot_Number<0)
	{
	  Remain_Shoot_Number=0;
	}
}

void Deal_Data_From_Chassis2( receive_from_chassis *FromC,uint8_t *msgData)
{
	FromC->now_speed =(msgData[0]<<8)|msgData[1];
}


void JudgementDataCheck_Task(void)
{

static int Shoot_Control_Times=0;
static int Limit_Time=0;
	
Shoot_Control_Times=(Shoot_Control_Times+1)%5;
Shoot_Heat =shoot_control_consulation.shoot_heat;//当前枪口热量



if(Shoot_Heat>30000) {
Shoot_Heat=0;
}

Robot_Level = shoot_control_consulation.robot_level;
switch(Robot_Level)
{
case 0:{Shoot_Max_Heat = LEVEVL1_HEAT;}
break;

case 1:{Shoot_Max_Heat = LEVEVL1_HEAT;}
break;

case 2:{Shoot_Max_Heat = LEVEVL2_HEAT;}
break;

case 3:{Shoot_Max_Heat = LEVEVL3_HEAT;}
break;

default:
break;
}



if(Shoot_Control_Times==0)
{
if(MucGetShootSpeed()==Spd_High)
{
	Shoot_Speed=25;
	if(ShootWheelModeFlag==1)
	{
	Shoot_Speed=15;
	}

} 
else
{
Shoot_Speed=30;
}


/*暂时不想向裁判系统发东西 上次尝试放在main()里出错*/
//g_refereeSystemReceMesg.student_interative.ReceiveData.client_custom_data.data1=MucGetShootSpeed();



if(Shoot_Heat>=Shoot_Max_Heat*0.90)
{
  Heat_Limit=1;
}

else{
if( Remain_Shoot_Number<=3){	
Heat_Limit=1;
} 
else{
Heat_Limit=0;
}

}

}

if(Heat_Limit==1)
{
Limit_Time++;
}
else 
{
Limit_Time=0;
}


if(Limit_Time>5000)
{
Heat_Limit=0;
}

//g_refereeSystemReceMesg.student_interative.ReceiveData.client_custom_data.data2=Heat_Limit;



}







#include "muc_Shoot_Judge_Control.h"
#include "muc_hardware_can.h"
#include "muc_shoot_wheel.h"

#define LEVEVL1_HEAT 230//240
#define LEVEVL2_HEAT 350//360
#define LEVEVL3_HEAT 470//480

uint16_t Shoot_Heat;      //17mm弹丸热量
uint16_t Shoot_Last_Heat; //上一次17mm弹丸热量
uint8_t  Shoot_Speed;     //射速 (自己估计的最大值)
uint16_t Remain_Shoot_Number=0;  //剩余可发射子弹数
uint16_t Shoot_Max_Heat;  //当前热量上限
uint8_t  Robot_Level;   //步兵等级
uint8_t  Heat_Limit=0;  //热量限制射击标志




receive_from_chassis shoot_control_consulation={0};

void Deal_Data_From_Chassis( receive_from_chassis  *FromC, uint8_t * msgData)
{
FromC->shoot_heat =(msgData[0]<<8)|msgData[1];
FromC->robot_level=(msgData[2]<<8)|msgData[3];
FromC->remain_hp  =(msgData[4]<<8)|msgData[5];
FromC->max_hp	  =(msgData[6]<<8)|msgData[7];
}


void JudgementDataCheck_Task(void)
{
	static int Shoot_Control_Times=0;
		
	Shoot_Control_Times=(Shoot_Control_Times+1)%25;
		
	Shoot_Last_Heat= Shoot_Heat;
	Shoot_Heat =shoot_control_consulation.shoot_heat;//当前枪口热量
		
	if(Shoot_Heat>30000) {
	Shoot_Heat=Shoot_Last_Heat;
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
	if(MucGetShootSpeed()==Spd_High){
	Shoot_Speed=14;
	}

	else{Shoot_Speed=25;
	}

	if(Shoot_Control_Times==0)
	{
			/*难道说裁判系统不会更新枪口热量？*/
		if(  shoot_control_consulation.remain_hp <=( shoot_control_consulation.max_hp*0.2) ){
			Shoot_Heat =Shoot_Heat-80;
		}
		else{
			Shoot_Heat=Shoot_Heat-40;
		}		

		if(Shoot_Heat>Shoot_Max_Heat*0.90){
			Heat_Limit=1;
			}

		Remain_Shoot_Number=( (Shoot_Max_Heat-(uint16_t)Shoot_Heat)-40 )/Shoot_Speed;

		if( Remain_Shoot_Number<=5){	
			Heat_Limit=1;
			}

		else{
			Heat_Limit=0;
			}

	}

		Shoot_Last_Heat=Shoot_Heat;
}


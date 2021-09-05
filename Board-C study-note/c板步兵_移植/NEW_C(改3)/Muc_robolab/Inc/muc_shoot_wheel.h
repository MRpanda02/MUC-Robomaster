#ifndef _MUC_SHOOT_WHEEL_H__
#define _MUC_SHOOT_WHEEL_H__
#include "main.h"

//左右摩擦轮pid
#define  STIR_LEFT_SPEED_KP  5
#define  STIR_RIGHT_SPEED_KP  5

//摩擦轮转速

#define SHOOT_LEFT_CONFIG_FrictionWheelSpeed_HIGH  14000  //before 3000  //24
#define SHOOT_LEFT_CONFIG_FrictionWheelSpeed_LOW   10000  //18

#define SHOOT_RIGHT_CONFIG_FrictionWheelSpeed_HIGH  -14000  //before 3000  //24
#define SHOOT_RIGHT_CONFIG_FrictionWheelSpeed_LOW   -10000  //18

typedef enum
{
    Spd_Stop = 0,
	Spd_High = 1,
}ShootSpeedSta_t;

typedef struct
{
  int16_t real[8];
	uint8_t count;   //数据接收计数
	int16_t round_cnt;	//圈数
	int16_t ecd_value;	//经过处理后连续的编码器值
	int16_t sum;		//编码器值累积和		
	int16_t bias;		//初始编码器值	
	int16_t calc;
	int16_t position_calc;	//位置计算值
	int16_t calc_diff;	//两次编码器之间的差值
	int16_t calc_last;
	int16_t turns;
} ShootReceiveTypeDef;
void ShootAnalysisWay(void);

void Shoot_Speed_Data_deal(ShootReceiveTypeDef *Receive, uint8_t * msgData);
ShootSpeedSta_t MucGetShootSpeed( void );
void MucSetShootSpeed( ShootSpeedSta_t sta );
void Shoot_Loop_Task( void );
void MucShootInit( void );

extern int ShootWheelModeFlag;
extern ShootReceiveTypeDef Shoot_Left_Feedback;
extern ShootReceiveTypeDef Shoot_Right_Feedback;
extern uint16_t MouseLeftPressTime;
extern uint16_t MouseRightPressTime; 

#endif

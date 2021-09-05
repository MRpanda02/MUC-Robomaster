#ifndef _MUC_SHOOT_WHEEL_H__
#define _MUC_SHOOT_WHEEL_H__
#include "main.h"

//����Ħ����pid
#define  STIR_LEFT_SPEED_KP  5
#define  STIR_RIGHT_SPEED_KP  5

//Ħ����ת��

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
	uint8_t count;   //���ݽ��ռ���
	int16_t round_cnt;	//Ȧ��
	int16_t ecd_value;	//��������������ı�����ֵ
	int16_t sum;		//������ֵ�ۻ���		
	int16_t bias;		//��ʼ������ֵ	
	int16_t calc;
	int16_t position_calc;	//λ�ü���ֵ
	int16_t calc_diff;	//���α�����֮��Ĳ�ֵ
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

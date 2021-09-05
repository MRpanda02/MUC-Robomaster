#ifndef _MUC_STIR_WHEEL_H__
#define _MUC_STIR_WHEEL_H__

#include "main.h"

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
} StirReceiveTypeDef;

void Shooting_Speed_Data_deal(StirReceiveTypeDef *Receive, uint8_t *msgData);
void Position_Round_Data_deal(StirReceiveTypeDef *Receive, uint8_t *msgData );
void SMControlLoop_Task(void);
void MucStirWheelInit( void );
void Magazine_Init(void);
extern StirReceiveTypeDef Stir_speed_Feedback;
extern StirReceiveTypeDef Stir_position_Feedback;
extern int32_t StirSpeedOut;
extern uint16_t MouseLeftPressTime;
extern uint16_t MouseRightPressTime; 
#endif

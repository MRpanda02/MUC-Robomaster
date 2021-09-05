#ifndef _MUC_STIR_WHEEL_H__
#define _MUC_STIR_WHEEL_H__

#include "main.h"

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

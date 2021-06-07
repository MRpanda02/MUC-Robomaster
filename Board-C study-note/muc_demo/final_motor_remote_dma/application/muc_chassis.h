#ifndef _MUC_CHASSIS_H__
#define _MUC_CHASSIS_H__
#include "main.h"

/*自旋目标速度*/
#define 	FINAL_CIRCLE_SPEED 	20 

/* 使用遥控器拨杆时的放大比例 */
#define 	KP_Remote 	7 
#define 	KP_Rotate   16

//底盘速度控制PID
#define CMSpeed_Pid_NORMAL_KP 1.4
#define CMSpeed_Pid_NORMAL_KI 0.0
#define CMSpeed_Pid_NORMAL_KD 0.0

// 地盘跟随PID
#define CMRotatePid_KP  7.9
#define CMRotatePid_KI	0
#define CMRotatePid_KD	2000

// 底盘自旋PID
#define CMCirclePid_KP  7.9
#define CMCirclePid_KI	0
#define CMCirclePid_KD	0

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
} CMReceiveTypeDef;

void ChassisDataAnalysisWay(void);

void MucChassisInit( void );
void CMControlLoopTask(void);
void ChassisDealDataFromCan(CMReceiveTypeDef *v, uint8_t *msgData);
void CMControlForChassisLoopTask(void);

extern CMReceiveTypeDef CM1_Feedback;
extern CMReceiveTypeDef CM2_Feedback;
extern CMReceiveTypeDef CM3_Feedback;
extern CMReceiveTypeDef CM4_Feedback;


extern int Sta_To_Circle;
#endif

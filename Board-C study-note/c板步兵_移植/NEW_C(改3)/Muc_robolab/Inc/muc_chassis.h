#ifndef _MUC_CHASSIS_H__
#define _MUC_CHASSIS_H__
#include "main.h"

/*����Ŀ���ٶ�*/
#define 	FINAL_CIRCLE_SPEED 	20 

/* ʹ��ң��������ʱ�ķŴ���� */
#define 	KP_Remote 	7 
#define 	KP_Rotate   16

//�����ٶȿ���PID
#define CMSpeed_Pid_NORMAL_KP 1.4
#define CMSpeed_Pid_NORMAL_KI 0.0
#define CMSpeed_Pid_NORMAL_KD 0.0

// ���̸���PID
#define CMRotatePid_KP  7.9
#define CMRotatePid_KI	0
#define CMRotatePid_KD	2000

// ��������PID
#define CMCirclePid_KP  7.9
#define CMCirclePid_KI	0
#define CMCirclePid_KD	0

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

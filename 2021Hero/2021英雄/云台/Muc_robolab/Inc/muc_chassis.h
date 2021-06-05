#ifndef _MUC_CHASSIS_H__
#define _MUC_CHASSIS_H__
#include "main.h"

/* ʹ��ң��������ʱ�ķŴ���� */
#define 	KP_Remote 	7 
#define 	KP_Rotate   16

//�����ٶȿ���PID
#define CMSpeed_Pid_NORMAL_KP 1.4
#define CMSpeed_Pid_NORMAL_KI 0.0
#define CMSpeed_Pid_NORMAL_KD 0.0

// ������ӦPID
#define CMRotatePid_KP 0  //0.5  //3.5 //7  //7.9
#define CMRotatePid_KI	0.3  //0.5  //0.1
#define CMRotatePid_KD	3000 //3000


//���������ٶ���
#define CM_rotate_pid_Kp_Max 250//230
#define CM_rotate_pid_Kp_Min 50


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

void MucChassisInit( void );
void ChassisDealDataFromCan(CMReceiveTypeDef *v, uint8_t *msgData);
void CMControlForChassisLoopTask(void);
void SetChassisSpeedDataFromGM( void );

extern CMReceiveTypeDef CM1_Feedback;
extern CMReceiveTypeDef CM2_Feedback;
extern CMReceiveTypeDef CM3_Feedback;
extern CMReceiveTypeDef CM4_Feedback;

#endif

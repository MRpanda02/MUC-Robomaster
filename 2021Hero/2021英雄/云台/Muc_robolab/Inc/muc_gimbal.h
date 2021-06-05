#ifndef _MUC_GIMBAL_H__
#define _MUC_GIMBAL_H__
#include "main.h"

// �������  0 3��  1 4��
#define INFANROBOT 1   

#if (INFANROBOT==0)

// Yaw �����ٶȻ�PID
#define YawSpeedPid_KP	86
#define YawSpeedPid_KI	0
#define YawSpeedPid_KD	0




// Yaw ����λ�û�PID
#define YawPositionPid_KP	-10
#define YawPositionPid_KI	0
#define YawPositionPid_KD	0

// Pitch �����ٶȻ�PID
#define PitchSpeedPid_KP	170
#define PitchSpeedPid_KI	0
#define PitchSpeedPid_KD	0

// Pitch ����λ�û�PID
#define PitchPositionPid_KP	17
#define PitchPositionPid_KI	0
#define PitchPositionPid_KD	0


#elif(INFANROBOT==1)
// Yaw �����ٶȻ�PID
#define YawSpeedPid_KP	180//95	
#define YawSpeedPid_KI	0
#define YawSpeedPid_KD	0




// Yaw ����λ�û�PID
#define YawPositionPid_KP	-16.5//-11.5
#define YawPositionPid_KI	0
#define YawPositionPid_KD	0

// Pitch �����ٶȻ�PID
#define PitchSpeedPid_KP	-220//105
#define PitchSpeedPid_KI	0//0.05
#define PitchSpeedPid_KD	0//10

// Pitch ����λ�û�PID
#define PitchPositionPid_KP -5//30
#define PitchPositionPid_KI 0//0
#define PitchPositionPid_KD	0//-20

#endif



#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;											//�ٶ�
	float ecd_angle;											//�Ƕ�
}Encoder;

void GimbalInit( void );
void GimbalDealDataFromCan(volatile Encoder *v, uint8_t *msgData);

void EncoderProcess(volatile Encoder *v, uint8_t *msgData);//������ݴ�����

void GMControlLoopTask( void );
extern void AutoShootingAndWindmill(void);
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;

#endif  // _MUC_GIMBAL_H__

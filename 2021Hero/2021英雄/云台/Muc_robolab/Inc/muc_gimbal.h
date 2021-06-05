#ifndef _MUC_GIMBAL_H__
#define _MUC_GIMBAL_H__
#include "main.h"

// 步兵编号  0 3号  1 4号
#define INFANROBOT 1   

#if (INFANROBOT==0)

// Yaw 轴电机速度环PID
#define YawSpeedPid_KP	86
#define YawSpeedPid_KI	0
#define YawSpeedPid_KD	0




// Yaw 轴电机位置环PID
#define YawPositionPid_KP	-10
#define YawPositionPid_KI	0
#define YawPositionPid_KD	0

// Pitch 轴电机速度环PID
#define PitchSpeedPid_KP	170
#define PitchSpeedPid_KI	0
#define PitchSpeedPid_KD	0

// Pitch 轴电机位置环PID
#define PitchPositionPid_KP	17
#define PitchPositionPid_KI	0
#define PitchPositionPid_KD	0


#elif(INFANROBOT==1)
// Yaw 轴电机速度环PID
#define YawSpeedPid_KP	180//95	
#define YawSpeedPid_KI	0
#define YawSpeedPid_KD	0




// Yaw 轴电机位置环PID
#define YawPositionPid_KP	-16.5//-11.5
#define YawPositionPid_KI	0
#define YawPositionPid_KD	0

// Pitch 轴电机速度环PID
#define PitchSpeedPid_KP	-220//105
#define PitchSpeedPid_KI	0//0.05
#define PitchSpeedPid_KD	0//10

// Pitch 轴电机位置环PID
#define PitchPositionPid_KP -5//30
#define PitchPositionPid_KI 0//0
#define PitchPositionPid_KD	0//-20

#endif



#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	//buf，for filter
	int32_t round_cnt;										//圈数
	int32_t filter_rate;											//速度
	float ecd_angle;											//角度
}Encoder;

void GimbalInit( void );
void GimbalDealDataFromCan(volatile Encoder *v, uint8_t *msgData);

void EncoderProcess(volatile Encoder *v, uint8_t *msgData);//电机数据处理函数

void GMControlLoopTask( void );
extern void AutoShootingAndWindmill(void);
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;

#endif  // _MUC_GIMBAL_H__

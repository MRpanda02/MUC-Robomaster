#ifndef _MUC_GIMBAL_H__
#define _MUC_GIMBAL_H__
#include "main.h"
#include "muc_pid.h"
#include "muc_remote.h"
#include "muc_imu.h"
#include "muc_hardware_can.h"
#include "muc_stmflash.h"
#include "muc_hardware_beep.h"
#include "muc_hardware_key.h"
#include "muc_stir_wheel.h"
#include "muc_auto_shoot.h"
#include "muc_referee_system.h"




#define RATE_BUF_SIZE 6
typedef struct{
	short raw_current;                             //电机反馈电流值
	short feedback_speed_value;                    //反馈速度值    
	
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;							     	//上一次的编码器原始值
	int32_t ecd_value;                                      //经过处理后连续的编码器值
	int32_t diff;											//两次编码器之间的差值
	int32_t temp_count;                                     //计数用
	uint8_t buf_count;								        //滤波更新buf用
	int32_t ecd_bias;										//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	                    //buf，for filter
	int32_t round_cnt;										//圈数
	int32_t filter_rate;									//速度
	
	                        
	float ecd_angle;	              //角度
	
}Encoder;

extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;

extern int16_t GimbalYawMidOffset;
extern int16_t GimbalPitchMidOffset;


void GimbalInit( void ); //main初始
void GimbalDealDataFromCan(volatile Encoder *v, uint8_t *msgData, int16_t initData); //CAN接收
void Analysis_Way_Choose(CC_TypeDef *GimbalRC); //REMOTE接收
void GMControlLoopTask(CC_TypeDef *GimbalRC, int GimbalSta ); //GIMBAL主任务

#endif  // _MUC_GIMBAL_H__

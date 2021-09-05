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
	short raw_current;                             //�����������ֵ
	short feedback_speed_value;                    //�����ٶ�ֵ    
	
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;							     	//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                                      //��������������ı�����ֵ
	int32_t diff;											//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                                     //������
	uint8_t buf_count;								        //�˲�����buf��
	int32_t ecd_bias;										//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	                    //buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;									//�ٶ�
	
	                        
	float ecd_angle;	              //�Ƕ�
	
}Encoder;

extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;

extern int16_t GimbalYawMidOffset;
extern int16_t GimbalPitchMidOffset;


void GimbalInit( void ); //main��ʼ
void GimbalDealDataFromCan(volatile Encoder *v, uint8_t *msgData, int16_t initData); //CAN����
void Analysis_Way_Choose(CC_TypeDef *GimbalRC); //REMOTE����
void GMControlLoopTask(CC_TypeDef *GimbalRC, int GimbalSta ); //GIMBAL������

#endif  // _MUC_GIMBAL_H__

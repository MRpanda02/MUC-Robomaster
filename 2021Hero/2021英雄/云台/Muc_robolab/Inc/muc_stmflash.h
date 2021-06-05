#ifndef _MUC_STMFLASH_H__
#define _MUC_STMFLASH_H__

#include "main.h"

#define GIMBAL_CALI_DATA    0X1
#define MPU6050_CALI_DATA    0X2

#define FLASH_SAVE_ADDR  0X08111000 	//����FLASH �����ַ(����Ϊ4�ı���������������,Ҫ���ڱ�������ռ�õ�������.
										//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.
#define FLASH_VAL_ADDR  0X08140000 


typedef struct
{
	uint32_t head;	//֡ͷ 0Xf9f1
	
	uint16_t	dataLen;	//���ݳ���
    uint16_t	status;
    
    int16_t XGygoMpuCaliValue;  //������Z��У׼ֵ
    int16_t YGygoMpuCaliValue;
    int16_t ZGygoMpuCaliValue;
    
    int16_t GimbalYawMidPos;	//Yaw��������ֵ
	int16_t GimbalPitchMidPos;	//Pitch��������ֵ
    
	int16_t checkSum;	//У���
    
} flash_data_t;

#define FlashData_Header  0Xf9f1f2f3

#define FlashData_LEN 	sizeof(flash_data_t)	 		  	//���鳤��	

void CheckConfigParams( void );
uint8_t GetCaliDataSta( uint8_t pos );
void UpdateConfigParamsForMpu6050( int16_t XGygoMpuCaliValue, int16_t YGygoMpuCaliValue, int16_t ZGygoMpuCaliValue );
void UpdateConfigParamsForGimbal( int16_t GimbalPitchMidPos, int16_t GimbalYawMidPos );
void GetMpu6050CaliData( int16_t *XGygoMpuCaliValue, int16_t *YGygoMpuCaliValue, int16_t *ZGygoMpuCaliValue );
void GetGimbalCaliData( int16_t *gimbalPitchMidPos, int16_t *gimbalYawMidPos);

#endif

#ifndef _MUC_STMFLASH_H__
#define _MUC_STMFLASH_H__

#include "main.h"

#define GIMBAL_CALI_DATA    0X1
#define MPU6050_CALI_DATA    0X2

#define FLASH_SAVE_ADDR  0X08111000 	//设置FLASH 保存地址(必须为4的倍数，且所在扇区,要大于本代码所占用到的扇区.
										//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.
#define FLASH_VAL_ADDR  0X08140000 


typedef struct
{
	uint32_t head;	//帧头 0Xf9f1
	
	uint16_t	dataLen;	//数据长度
    uint16_t	status;
    
    int16_t XGygoMpuCaliValue;  //陀螺仪Z轴校准值
    int16_t YGygoMpuCaliValue;
    int16_t ZGygoMpuCaliValue;
    
    int16_t GimbalYawMidPos;	//Yaw轴电机中心值
	int16_t GimbalPitchMidPos;	//Pitch轴电机中心值
    
	int16_t checkSum;	//校验和
    
} flash_data_t;

#define FlashData_Header  0Xf9f1f2f3

#define FlashData_LEN 	sizeof(flash_data_t)	 		  	//数组长度	

void CheckConfigParams( void );
uint8_t GetCaliDataSta( uint8_t pos );
void UpdateConfigParamsForMpu6050( int16_t XGygoMpuCaliValue, int16_t YGygoMpuCaliValue, int16_t ZGygoMpuCaliValue );
void UpdateConfigParamsForGimbal( int16_t GimbalPitchMidPos, int16_t GimbalYawMidPos );
void GetMpu6050CaliData( int16_t *XGygoMpuCaliValue, int16_t *YGygoMpuCaliValue, int16_t *ZGygoMpuCaliValue );
void GetGimbalCaliData( int16_t *gimbalPitchMidPos, int16_t *gimbalYawMidPos);

#endif

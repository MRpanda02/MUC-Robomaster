#ifndef _MPU6500_H__
#define _MPU6500_H__

#include "main.h"

#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

#define IST8310_NSS_Low() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET) 	
#define IST8310_NSS_High() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET)

#define M_PI  (float)3.1415926535
	
typedef struct __MPU6500_REAL_Data__
{
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
    float Temp;     //转换成实际的温度，单位为摄氏度
    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度
	float Mag_X;   //转换成实际的X轴角加速度，
    float Mag_Y;   //转换成实际的Y轴角加速度，
    float Mag_Z;   //转换成实际的Z轴角加速度
	
}MPU6500_REAL_DATA;

typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;

uint8_t MPU6500_Init(void);
uint8_t IST8310_Init(void);
void IMU_Get_Data( void );

extern IMUDataTypedef imu_data;
extern IMUDataTypedef imu_raw_data;
extern IMUDataTypedef imu_data_offest;
extern volatile MPU6500_REAL_DATA   MPU6500_Real_Data;

#endif

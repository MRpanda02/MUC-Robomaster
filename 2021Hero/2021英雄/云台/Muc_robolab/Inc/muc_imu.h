#ifndef _MUC_IMU_H__
#define _MUC_IMU_H__

#include "main.h"
#include "mpu6500.h"

#define IMU_Cali_COUNTS 5000

void MucImuInit( void );
void GetMpu6500DataLoop( void );
void ImuAnalysisTask( void );

extern volatile char isMPU6050_is_DRY;
extern volatile float Mpu6500AngleData;

#endif


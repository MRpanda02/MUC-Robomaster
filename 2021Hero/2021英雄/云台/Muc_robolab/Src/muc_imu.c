#include "muc_imu.h"
#include "mpu6500.h"
#include "muc_hardware_key.h"
#include "muc_stmflash.h"
#include "muc_hardware_beep.h"
#include "muc_gimbal.h"

volatile float Mpu6500AngleData = 0;

volatile char isMPU6050_is_DRY = 0;
volatile float mygetqval[9];	//用于存放传感器转换结果的数组
int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值

/* 更新MPU6050的误差数据，如果没有有效数据，则会提示更新 */
void UpdateMpu6500GyroCaliData( void )
{
    int i;
    int64_t mpuRawGyroDataTmp[3];
    if(GetCaliDataSta(MPU6050_CALI_DATA))
    {
        // 数据有效
        GetMpu6050CaliData( &imu_data_offest.gx, &imu_data_offest.gy, &imu_data_offest.gz);
        
    }else{
        
        /* 重新校准 按下白色按钮开始校准 */
        bibiSing(4); /* 报错提示 */
        while(!GetKeyPressUpAction())
        {
            HAL_Delay(100);
        }
        bibiSing(2);
        
        HAL_Delay(1000);
        HAL_Delay(1000);
        HAL_Delay(1000);

        
        bibiSing(1);
        for( i=0; i<IMU_Cali_COUNTS; i++ )
        {
            IMU_Get_Data();
            mpuRawGyroDataTmp[0] += imu_raw_data.gx;
            mpuRawGyroDataTmp[1] += imu_raw_data.gy;
            mpuRawGyroDataTmp[2] += imu_raw_data.gz;
        }
        /* 校准角速度误差 */
        mpuRawGyroDataTmp[0]=0;
        mpuRawGyroDataTmp[1]=0;
        mpuRawGyroDataTmp[2]=0;
        for( i=0; i<IMU_Cali_COUNTS; i++ )
        {
            IMU_Get_Data();
            mpuRawGyroDataTmp[0] += imu_raw_data.gx;
            mpuRawGyroDataTmp[1] += imu_raw_data.gy;
            mpuRawGyroDataTmp[2] += imu_raw_data.gz;
        }
        /* 完成校准 存储数据 */
        UpdateConfigParamsForMpu6050( mpuRawGyroDataTmp[0]/IMU_Cali_COUNTS, mpuRawGyroDataTmp[1]/IMU_Cali_COUNTS, mpuRawGyroDataTmp[2]/IMU_Cali_COUNTS );
        GetMpu6050CaliData( &imu_data_offest.gx, &imu_data_offest.gy, &imu_data_offest.gz);
        bibiSing(2);
    }
}

void MucImuInit( void )
{
    /* 初始化陀螺仪和加速度计 */
    MPU6500_Init();

    /* 初始化磁力计 */
    IST8310_Init();	
    
    UpdateMpu6500GyroCaliData();

}

void GetMpu6500DataLoop( void )
{
    IMU_Get_Data();
    isMPU6050_is_DRY = 1;
}


/**********************************************************************************/
/*将MPU6050_ax,MPU6050_ay, MPU6050_az,MPU6050_gx, MPU6050_gy, MPU6050_gz处理后存储*/
/**********************************************************************************/
void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]为最近10次数据 [10]为10次数据的平均值
{
    static uint8_t nowIndex=0;
	uint8_t i = 0;
	int32_t sum=0;
	
	MPU6050_FIFO[0][nowIndex]=ax;//将新的数据放置到 数据的最后面
	MPU6050_FIFO[1][nowIndex]=ay;
	MPU6050_FIFO[2][nowIndex]=az;
	MPU6050_FIFO[3][nowIndex]=gx;
	MPU6050_FIFO[4][nowIndex]=gy;
	MPU6050_FIFO[5][nowIndex]=gz;
    nowIndex = (nowIndex+1)%10;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
	
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
*功　　能:	    读取 MPU6050的当前测量值
*******************************************************************************/
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) 
{
    int16_t MPU6050_Lastax=0;
    int16_t MPU6050_Lastay=0;
    int16_t MPU6050_Lastaz=0;
    int16_t MPU6050_Lastgx=0;
    int16_t MPU6050_Lastgy=0;
    int16_t MPU6050_Lastgz=0;
    
    if(isMPU6050_is_DRY)
    {
        isMPU6050_is_DRY = 0;
        //读取加速度和陀螺仪的当前ADC

        //MPU6050_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,mpu_buf,14);  //
        //HMC58X3_ReadData(&(mpu_buf[14]));  //14-19为陀螺仪数据
        MPU6050_Lastax=imu_data.ax;
        MPU6050_Lastay=imu_data.ay;
        MPU6050_Lastaz=imu_data.az;
        //跳过温度ADC
        MPU6050_Lastgx=imu_data.gx;
        MPU6050_Lastgy=imu_data.gy;
        MPU6050_Lastgz=imu_data.gz;

        MPU6050_DataSave(MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz);  		
        *ax  =MPU6050_FIFO[0][10];
        *ay  =MPU6050_FIFO[1][10];
        *az = MPU6050_FIFO[2][10];
        *gx  =MPU6050_FIFO[3][10];// - GyroSavedCaliData.GyroXOffset;
        *gy = MPU6050_FIFO[4][10];// - GyroSavedCaliData.GyroYOffset;
        *gz = MPU6050_FIFO[5][10];
    } else {       //读取上一次的值
        *ax = MPU6050_FIFO[0][10];//=MPU6050_FIFO[0][10];
        *ay = MPU6050_FIFO[1][10];//=MPU6050_FIFO[1][10];
        *az = MPU6050_FIFO[2][10];//=MPU6050_FIFO[2][10];
        *gx = MPU6050_FIFO[3][10];//-GyroSavedCaliData.GyroXOffset;//=MPU6050_FIFO[3][10];
        *gy = MPU6050_FIFO[4][10];//-GyroSavedCaliData.GyroYOffset;//=MPU6050_FIFO[4][10];
        *gz = MPU6050_FIFO[5][10];//-GyroSavedCaliData.GyroZOffset;//=MPU6050_FIFO[5][10];
    }
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(volatile float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值  
输入参数： 将结果存放的数组首地址
加速度值：原始数据，-8192-+8192
角速度值：deg/s
磁力计值：原始数据
输出参数：没有
*******************************************************************************/
void IMU_getValues(volatile float * values) 
{  
    int16_t accgyroval[6];
    int gyroTemp=0;
    int i;

    MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);

    for(i = 0; i<6; i++) 
    {
        if(i < 3) {
            values[i] = (float) accgyroval[i];
        } else {
            //accgyroval[5]=9;
            gyroTemp = (int)((accgyroval[i]) / 32.8f); //转成度每秒
            values[i] = (float)gyroTemp;
            //这里已经将量程改成了 1000度每秒  32.8 对应 1度每秒
        }
    }
    values[6] = imu_data.mx;
    values[7] = imu_data.my;
    values[8] = imu_data.mz;
  
}


void ImuAnalysisTask( void )
{
    static uint8_t firstRun = 1;
    IMU_getValues(mygetqval);	 //获取原始数据,加速度计和磁力计是原始值，陀螺仪转换成了deg/s
    
	MPU6500_Real_Data.Gyro_X = mygetqval[3];
	MPU6500_Real_Data.Gyro_Y = -mygetqval[4];
	MPU6500_Real_Data.Gyro_Z = mygetqval[5];
    /* 将云台电机的YAW偏角作为磁力计的初始角度,为了解决开机时地盘不动云台回中的问题. */
    if( firstRun )
    {
        Mpu6500AngleData = -GMYawEncoder.ecd_angle;
        firstRun = 0;
    }
	Mpu6500AngleData -= MPU6500_Real_Data.Gyro_Z*0.001f;
}

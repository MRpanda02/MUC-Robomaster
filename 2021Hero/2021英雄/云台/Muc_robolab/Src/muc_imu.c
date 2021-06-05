#include "muc_imu.h"
#include "mpu6500.h"
#include "muc_hardware_key.h"
#include "muc_stmflash.h"
#include "muc_hardware_beep.h"
#include "muc_gimbal.h"

volatile float Mpu6500AngleData = 0;

volatile char isMPU6050_is_DRY = 0;
volatile float mygetqval[9];	//���ڴ�Ŵ�����ת�����������
int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ

/* ����MPU6050��������ݣ����û����Ч���ݣ������ʾ���� */
void UpdateMpu6500GyroCaliData( void )
{
    int i;
    int64_t mpuRawGyroDataTmp[3];
    if(GetCaliDataSta(MPU6050_CALI_DATA))
    {
        // ������Ч
        GetMpu6050CaliData( &imu_data_offest.gx, &imu_data_offest.gy, &imu_data_offest.gz);
        
    }else{
        
        /* ����У׼ ���°�ɫ��ť��ʼУ׼ */
        bibiSing(4); /* ������ʾ */
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
        /* У׼���ٶ���� */
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
        /* ���У׼ �洢���� */
        UpdateConfigParamsForMpu6050( mpuRawGyroDataTmp[0]/IMU_Cali_COUNTS, mpuRawGyroDataTmp[1]/IMU_Cali_COUNTS, mpuRawGyroDataTmp[2]/IMU_Cali_COUNTS );
        GetMpu6050CaliData( &imu_data_offest.gx, &imu_data_offest.gy, &imu_data_offest.gz);
        bibiSing(2);
    }
}

void MucImuInit( void )
{
    /* ��ʼ�������Ǻͼ��ٶȼ� */
    MPU6500_Init();

    /* ��ʼ�������� */
    IST8310_Init();	
    
    UpdateMpu6500GyroCaliData();

}

void GetMpu6500DataLoop( void )
{
    IMU_Get_Data();
    isMPU6050_is_DRY = 1;
}


/**********************************************************************************/
/*��MPU6050_ax,MPU6050_ay, MPU6050_az,MPU6050_gx, MPU6050_gy, MPU6050_gz�����洢*/
/**********************************************************************************/
void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
{
    static uint8_t nowIndex=0;
	uint8_t i = 0;
	int32_t sum=0;
	
	MPU6050_FIFO[0][nowIndex]=ax;//���µ����ݷ��õ� ���ݵ������
	MPU6050_FIFO[1][nowIndex]=ay;
	MPU6050_FIFO[2][nowIndex]=az;
	MPU6050_FIFO[3][nowIndex]=gx;
	MPU6050_FIFO[4][nowIndex]=gy;
	MPU6050_FIFO[5][nowIndex]=gz;
    nowIndex = (nowIndex+1)%10;
	
	for(i=0;i<10;i++)//��ǰ����ĺϣ���ȡƽ��ֵ
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
*��������:	    ��ȡ MPU6050�ĵ�ǰ����ֵ
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
        //��ȡ���ٶȺ������ǵĵ�ǰADC

        //MPU6050_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,mpu_buf,14);  //
        //HMC58X3_ReadData(&(mpu_buf[14]));  //14-19Ϊ����������
        MPU6050_Lastax=imu_data.ax;
        MPU6050_Lastay=imu_data.ay;
        MPU6050_Lastaz=imu_data.az;
        //�����¶�ADC
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
    } else {       //��ȡ��һ�ε�ֵ
        *ax = MPU6050_FIFO[0][10];//=MPU6050_FIFO[0][10];
        *ay = MPU6050_FIFO[1][10];//=MPU6050_FIFO[1][10];
        *az = MPU6050_FIFO[2][10];//=MPU6050_FIFO[2][10];
        *gx = MPU6050_FIFO[3][10];//-GyroSavedCaliData.GyroXOffset;//=MPU6050_FIFO[3][10];
        *gy = MPU6050_FIFO[4][10];//-GyroSavedCaliData.GyroYOffset;//=MPU6050_FIFO[4][10];
        *gz = MPU6050_FIFO[5][10];//-GyroSavedCaliData.GyroZOffset;//=MPU6050_FIFO[5][10];
    }
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void IMU_getValues(volatile float * values)
*��������:	 ��ȡ���ٶ� ������ ������ �ĵ�ǰֵ  
��������� �������ŵ������׵�ַ
���ٶ�ֵ��ԭʼ���ݣ�-8192-+8192
���ٶ�ֵ��deg/s
������ֵ��ԭʼ����
���������û��
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
            gyroTemp = (int)((accgyroval[i]) / 32.8f); //ת�ɶ�ÿ��
            values[i] = (float)gyroTemp;
            //�����Ѿ������̸ĳ��� 1000��ÿ��  32.8 ��Ӧ 1��ÿ��
        }
    }
    values[6] = imu_data.mx;
    values[7] = imu_data.my;
    values[8] = imu_data.mz;
  
}


void ImuAnalysisTask( void )
{
    static uint8_t firstRun = 1;
    IMU_getValues(mygetqval);	 //��ȡԭʼ����,���ٶȼƺʹ�������ԭʼֵ��������ת������deg/s
    
	MPU6500_Real_Data.Gyro_X = mygetqval[3];
	MPU6500_Real_Data.Gyro_Y = -mygetqval[4];
	MPU6500_Real_Data.Gyro_Z = mygetqval[5];
    /* ����̨�����YAWƫ����Ϊ�����Ƶĳ�ʼ�Ƕ�,Ϊ�˽������ʱ���̲�����̨���е�����. */
    if( firstRun )
    {
        Mpu6500AngleData = -GMYawEncoder.ecd_angle;
        firstRun = 0;
    }
	Mpu6500AngleData -= MPU6500_Real_Data.Gyro_Z*0.001f;
}

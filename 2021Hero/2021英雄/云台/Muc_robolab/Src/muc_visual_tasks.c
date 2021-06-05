#include "muc_visual_tasks.h"
#include "muc_hardware_uart.h"
#include "muc_gimbal.h"
#include "muc_remote.h"


uint8_t g_MiniPCRxBuf[MINIPC_BUF_LEN]={0};
int Data[8] = {0};
int shoot_command = 0;
int8_t toward_follow = 1;

Shooting_orientation Version_Shooting = {0,0,0}; 
int check = 0;
uint8_t out_view = 1;
int out_times = 0;
void Dma_Callback_MS_Handle(Shooting_orientation*View,uint8_t* buff)//处理后的数据，收到的数据
{ 

		for(int i=0;i<8;i++)
		 {
			 Data[i] = buff[i]-48;
		 }
		 check = Data[3]+Data[6];
		 check = check%10;
		if(check == Data[7])
	{
		 if(check == 0)//无敌人
		 {
			 if(Data[0] == 0)
		  {
				 out_times++;
				 if(out_times > 200)
				 {
					 out_view = 1;
				 }
		  }
		 }
		 else
			 {
				 out_view = 0;
				 check=check%10;
					if(Data[1] == 0)
					{
						Data[1] = -1;
					}
					if(Data[4] == 0)
					{
						Data[4] = -1;
					}

				 View->attackCount = Data[0];
				 View->Yaw_orientation = Data[1]*(Data[2]*10+Data[3]);//-60 ~ 60度
				 View->Pitch_orientation = Data[4]*(Data[5]*10+Data[6]);//-50 ~ 50度
		  }
		}
}

float Butterworth_Filter(float x)
{
	float a = 0.92f;
	float Y = 0;
	 Y = a*x + (1-a)*Y;
	return Y;
}
float last_pitchangle = 0;
float last_yawangle = 0;
float out_time = -1;
float Yaw_differential = 0;
float Pitch_differential = 0;
int shooting_time= -1;
int lost_flag =0;
void AutoShootingAndWindmill(void) //自瞄函数
{
	Version_Shooting.Yaw_orientation = Butterworth_Filter(Version_Shooting.Yaw_orientation);
	Version_Shooting.Pitch_orientation =  Butterworth_Filter(Version_Shooting.Pitch_orientation);
	if(out_view == 0)  //有敌人
	{
		
		CC_Condition.yaw_angle_ref = Mpu6500AngleData + Version_Shooting.Yaw_orientation/1.6;
		CC_Condition.pitch_angle_ref = GMPitchEncoder.ecd_angle + Version_Shooting.Pitch_orientation/3;
	}
	
}
void MucAutoShootInit(void)
{
	
	MucUartReceiveIT(&MiniPC_Uart,g_MiniPCRxBuf,MINIPC_BUF_LEN);
	HAL_UART_Receive_DMA(&MiniPC_Uart,g_MiniPCRxBuf,MINIPC_BUF_LEN);
}

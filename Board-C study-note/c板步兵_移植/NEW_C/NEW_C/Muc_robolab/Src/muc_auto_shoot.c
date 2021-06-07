#include "muc_auto_shoot.h"
#include "muc_hardware_uart.h"
#include "muc_pid.h"

#include "math.h"
#include "muc_Kalman.h"

kalman2_state VisionState={0};

#define KALMAN_Q 0.02
#define KALMAN_R 7.0000

#define VISION_SHOOT_YAW_KP 0.3//0.15
#define VISION_SHOOT_YAW_KI 0
#define VISION_SHOOT_YAW_KD 0

#define VISION_SHOOT_PITCH_KP 0.09//0.15
#define VISION_SHOOT_PITCH_KI 0
#define VISION_SHOOT_PITCH_KD 0.15

//typedef struct {
//    float x[2];     /* state: [0]-angle [1]-diffrence of angle, 2x1 */
//    float A[2][2];  /* X(n)=A*X(n-1)+U(n),U(n)~N(0,q), 2x2 */
//    float H[2];     /* Z(n)=H*X(n)+W(n),W(n)~N(0,r), 1x2   */
//    float q[2];     /* process(predict) noise convariance,2x1 [q0,0; 0,q1] */
//    float r;        /* measure noise convariance */
//    float p[2][2];  /* estimated error convariance,2x2 [p0 p1; p2 p3] */
//    float gain[2];  /* 2x1 */
//} kalman2_state;      

// ����ƽ���˲������ֳƻ���ƽ���˲�����
#define FILTER_N 12
int filter_buf[FILTER_N + 1];
int Filter(double RawData) 
{
int i;
int filter_sum = 0;
filter_buf[FILTER_N] = RawData; //ADת����ֵ�����������һ��ֵ
for(i = 0; i < FILTER_N; i++) 
{
filter_buf[i] = filter_buf[i + 1]; // �����������ƣ���λ�Ե�
filter_sum += filter_buf[i];
}
return (int)(filter_sum / FILTER_N);
} 


static double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
	
    double R = MeasureNoise_R;
    double Q = ProcessNiose_Q;

    static double x_last;
    double x_mid = x_last;
    double x_now;

    static double p_last;
    double p_mid ;
    double p_now;

    double kg;

    x_mid=x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����

    /*
     *  �������˲��������Ҫ��ʽ
     */
    kg=p_mid/(p_mid+R);                 //kgΪkalman filter��R Ϊ����
    x_now=x_mid+kg*(ResrcData-x_mid);   //���Ƴ�������ֵ
    p_now=(1-kg)*p_mid;                 //����ֵ��Ӧ��covariance
    p_last = p_now;                     //����covariance ֵ
    x_last = x_now;                     //����ϵͳ״ֵ̬

    return x_now;
}


/*���������м�洢��,ֱ�Ӹ�ֵ����ѧ����*/
int Vision_Task_Windmill=0;
int Vision_Task_Autoshooting=0;


double VisionYawRecive[3]={0};
double VisionPitchRecive[3]={0};

double VisionYawTarget=0.0f;
double VisionPitchTarget=0.0f;


int VisionChecking=0;

PidTypeDef Vision_Yaw_Target_Pid={0};
PidTypeDef Vision_Pitch_Target_Pid={0};


unsigned char g_MiniPCRxBuf[MINIPC_BUF_LEN]={0};
PidTypeDef Vision_Yaw_pid={0};

void MucAutoShootInit( void )
{
MucUartReceiveIT(&MiniPC_Uart, g_MiniPCRxBuf, MINIPC_BUF_LEN);	
PID_Init(&Vision_Yaw_Target_Pid,VISION_SHOOT_YAW_KP,VISION_SHOOT_YAW_KI,VISION_SHOOT_YAW_KD,0,16000,0,0,16000);
PID_Init(&Vision_Pitch_Target_Pid,VISION_SHOOT_PITCH_KP,VISION_SHOOT_PITCH_KI,VISION_SHOOT_PITCH_KD,0,16000,0,0,16000);
//kalman2_init(&VisionState, yaw, &yawspeed);
	
	PID_Init(&Vision_Yaw_pid,0,0,0,0,30000,0,0,0x7ffffff);
}




void Data_Deal(uint16_t Size)
{
int write=0;
int inter[8]={0};	
for(int i=0;i<30;i++)
{
if (g_MiniPCRxBuf[i]=='a')
{
	int o=0;
	write=i;
	for(int j=write;j<(write+8);j++)
	 {
	
        inter[j-write]=(int)g_MiniPCRxBuf[j]-48;
	 }    
	i=40;
	 MucDmaCallbackMiniPCHandle( inter,Size); 
}

}
		

}
int VisionReceiveFlag=0;
int VisionCheck=0;
void MucDmaCallbackMiniPCHandle( int* final,uint16_t size )
{
VisionCheck=(VisionCheck+1)%2;
/*������͵�����Ч���ݣ�����*/
	VisionPitchTarget=0;
		VisionYawTarget=0;
int Check=0;
    Check=final[3]+final[6];
	Check= Check%10;
	
/*�������Ч���ݶ���ͨ��У��*/
	if( Check==final[7])
	{
	VisionReceiveFlag=1;
    VisionYawRecive[0]=(int)final[1];//����
    VisionYawRecive[1]=(int)final[2]*10;//yawʮλ
    VisionYawRecive[2]=(int)final[3];//yaw��λ
		

		
	if(VisionYawRecive[0]==0)
	{
	VisionYawTarget=  -VisionYawRecive[1]- VisionYawRecive[2];

	}
		else
		{
		VisionYawTarget=  ( VisionYawRecive[1]+  VisionYawRecive[2]);
		
		}


    VisionPitchRecive[0]=(double)final[4];
	
	
    VisionPitchRecive[1]=(double)final[5]*10;
    VisionPitchRecive[2]=(double)final[6];	
				
	
	if(VisionPitchRecive[0]==0)
	{
	
	VisionPitchTarget=  -VisionPitchRecive[1]- VisionPitchRecive[2];
	}
		else
		{
		
		VisionPitchTarget=  VisionPitchRecive[1] +VisionPitchRecive[2];
		}


	}
	
	else
	{
	VisionReceiveFlag=0;
	
	}
/*У��ͼ���ʱ����*/	
    Check=0;
	
}

void AutoShootingAndWindmill(CC_TypeDef *GimbalRC)
{
	if(VisionCheck==0)
	{
		
		if(VisionReceiveFlag==1) 
		{
//			if(VisionYawTarget!=0)
//			{	
			
//		 VAL_LIMIT( VisionYawTarget,-10,10 );

       GimbalRC->yaw_angle_ref+=VisionYawTarget*0.1;
				
				
	
//		 VAL_LIMIT( VisionPitchTarget,-50,50 );

       GimbalRC->pitch_angle_ref+=VisionPitchTarget*0.1;
				
//			}
			
			
			
			
		}
//	Vision_Yaw_pid.Kp=0.06;
//	 
//     PID_Calc( &Vision_Yaw_pid,VisionYawTarget,0);
//	GimbalRC->yaw_angle_ref += Vision_Yaw_pid.output_compensation;
		VisionYawTarget=0;
		VisionPitchTarget=0;
	}

}
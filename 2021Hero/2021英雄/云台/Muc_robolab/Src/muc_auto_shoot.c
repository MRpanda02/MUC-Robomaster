#include "muc_auto_shoot.h"
#include "muc_hardware_uart.h"
#include "muc_pid.h"
#include "muc_remote.h"
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




double VisionYawRecive[8];
double VisionPitchRecive[8];
double VisionDistanceRecive[5];


double AutoShootPitchDirection=0;
double TargetDistance=0;

PidTypeDef Vision_Yaw_Target_Pid={0};
PidTypeDef Vision_Pitch_Target_Pid={0};


//unsigned char g_MiniPCRxBuf[MINIPC_BUF_LEN]={0};


float yaw[2]={0};
float yawspeed[2]={1,1};

void MucAutoShootInit( void )
{
MucUartReceiveIT(&MiniPC_Uart, g_MiniPCRxBuf, MINIPC_BUF_LEN);	
//kalman2_init(&VisionState, yaw, &yawspeed);
}




int time3;
void Data_Deal(uint16_t Size)
{
time3++;
int write=0;
char inter[11]={0};	
for(int i=0;i<30;i++)
{
if (g_MiniPCRxBuf[i]=='a')
{
	write=i;
	for(int j=write;j<write+11;j++)
	 {
        inter[j-write]=g_MiniPCRxBuf[j];
	 }    
	i=40;
}
}
		
MucDmaCallbackMiniPCHandle( inter,Size);
}
 
int time1=0;
int time2=0;
double Obj_yaw_Kd;
double Obj_pitch_Kd;
extern double pitch_angle_pos_ecd;
void MucDmaCallbackMiniPCHandle( char* final,uint16_t size )
{
#define ButYaw 325  //Yaw��У׼
#define ButPitch -170 //Pitch��У׼
time2++;
static int Check=0;	

for(int i=1;i<7;i++)
{
  Check+=final[i]-48;
}	

/*������͵�����Ч���ݣ�����*/
	if (Check==0){
	return;
	}
	else{	
	Check= Check%10;
	}

/*�������Ч���ݶ���ͨ��У��*/
	if( Check==final[10]-48)
	{
		static double real_yaw[3]={0};
    static double real_pitch[3]={0};
    
    VisionYawRecive[0]=(int)(final[1]-48)*100;
    VisionYawRecive[1]=(int)(final[2]-48)*10;
    VisionYawRecive[2]=(int)(final[3]-48);
    VisionYawRecive[3]=VisionYawRecive[0]+VisionYawRecive[1]+VisionYawRecive[2]-ButYaw;
    
    real_yaw[2]=real_yaw[1];
    real_yaw[1]=real_yaw[0];
    real_yaw[0]=VisionYawRecive[3];
    Obj_yaw_Kd = 0.1*(real_yaw[0] - real_yaw[1]);
    if(Obj_yaw_Kd <= 10 ) Obj_yaw_Kd = 10;
    if(MyAbs( VisionYawRecive[3])<Obj_yaw_Kd)
    {
      CC_Condition.yaw_angle_ref = Mpu6500AngleData;
    }
    
    VisionPitchRecive[0]=(double)(final[4]-48)*100;
    VisionPitchRecive[1]=(double)(final[5]-48)*10;
    VisionPitchRecive[2]=(double)(final[6]-48);
    VisionPitchRecive[3]=VisionPitchRecive[0]+VisionPitchRecive[1]+VisionPitchRecive[2]-ButPitch;

    real_pitch[2]=real_pitch[1];
    real_pitch[1]=real_pitch[0];
    real_pitch[0]=VisionPitchRecive[3];
    Obj_pitch_Kd = 0.1*(real_pitch[0] - real_pitch[1]);
    if(Obj_pitch_Kd <= 1 ) Obj_pitch_Kd = 1;
    if(MyAbs( VisionPitchRecive[3])<Obj_pitch_Kd)
    {
      CC_Condition.pitch_angle_ref = pitch_angle_pos_ecd;
    }
    
    VisionDistanceRecive[0]=(double)(final[7]-48)*100;
    VisionDistanceRecive[1]=(double)(final[8]-48)*10;
    VisionDistanceRecive[2]=(double)(final[9]-48);
    VisionDistanceRecive[3]=VisionDistanceRecive[0]+VisionDistanceRecive[1]+VisionDistanceRecive[2];
		
		
		
    time1++;
	}
	
/*У��ͼ���ʱ����*/	
    Check=0;
	
}

void AutoShootingAndWindmill(void)
{
	double final_yaw=0;
	double final_pitch=0;
	static double real_yaw[2]={0};
	static double real_pitch[2]={0};

	 real_yaw[1] =real_yaw[0];
	 real_yaw[0]=VisionYawRecive[3];
  
   real_pitch[1] =real_pitch[0];
	 real_pitch[0]=VisionPitchRecive[3];

	if(MyAbs(real_yaw[0]) >= Obj_yaw_Kd){
		final_yaw = real_yaw[0]*10.0; //0.50
		
		CC_Condition.yaw_angle_ref += final_yaw;
		double Yaw_angKd = MyAbs(real_yaw[0])*0.12;
		VAL_LIMIT( CC_Condition.yaw_angle_ref, Mpu6500AngleData - Yaw_angKd,Mpu6500AngleData + Yaw_angKd );
	}
  
  if(MyAbs(real_pitch[0]) >= Obj_pitch_Kd){
		final_pitch = real_pitch[0]*10; //0.50
		
		CC_Condition.pitch_angle_ref += final_pitch;
		double Pitch_angKd = MyAbs(real_pitch[0])*0.10;
		VAL_LIMIT( CC_Condition.pitch_angle_ref, pitch_angle_pos_ecd - Pitch_angKd,pitch_angle_pos_ecd + Pitch_angKd );
	}

	VisionPitchRecive[3]=0;
	VisionYawRecive[3]=0;

	
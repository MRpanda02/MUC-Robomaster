#ifndef _MUC_REMOTE_H__
#define _MUC_REMOTE_H__
#include "main.h"

/* 模块状态 */
typedef enum
{
	RC_OffLINE, 
	RC_ONLINE
	
}RcSta_t;


#define  Key_W_FLAG 	1
#define  Key_S_FLAG 	2
#define  Key_A_FLAG 	4
#define  Key_D_FLAG 	8

#define  Key_SHIFT_FLAG 	16
#define  Key_CTRL_FLAG 		32

#define  Key_Q_FLAG 	64
#define  Key_E_FLAG 	128
#define  Key_R_FLAG 	256
#define  Key_F_FLAG 	512

#define  Key_G_FLAG 	1024
#define  Key_Z_FLAG 	2048
#define  Key_X_FLAG 	4096
#define  Key_C_FLAG 	8192
#define  Key_V_FLAG 	16384
#define  Key_B_FLAG   32768


/* 遥控器拨杆状态 */
typedef enum
{
	NoAction=0,
	SwitchAction_1TO3=1,
	SwitchAction_2TO3=2, 
	SwitchAction_3TO1=3,
	SwitchAction_3TO2=4,
}SwitchAction;

typedef enum
{
	PressDown=0,
	PressUp=1,
}KeyBoardSta;

typedef enum
{
	StaN0=0,
	StaN1=1,
	StaN2=2, 
	StaN3=3,
}SwitchState;


/** 
  * @brief  遥控数据结构体
  */
typedef struct{
	//遥控器通道
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	uint8_t s1;  //switch_left
	uint8_t s2;	 //switch_right
	//鼠标
	struct{
		int16_t x;
		int16_t y;
		int16_t z;
		
		uint16_t press_left;
		uint16_t press_right;

	}mouse;
	//键盘
	uint16_t key_code;

}RC_TypeDef;

/** 
  * @brief  控制目标结构体
  */
typedef struct{
    
    RcSta_t rcSta;

	
	//移动方向
	int16_t FrontBack;
	int16_t LeftRight;
	int16_t Direction;
	
    float pitch_angle_ref;
    float yaw_angle_ref;
	
	SwitchAction S1Action;
	SwitchAction S2Action;
	
	SwitchState S1Sta;
	SwitchState S2Sta;
	
	KeyBoardSta MouseLeftBt;
	KeyBoardSta MouseRightBt;
	
	KeyBoardSta Key_W;	//1
	KeyBoardSta Key_S;	//2
	KeyBoardSta Key_A;	//4
	KeyBoardSta Key_D;	//8
	
	KeyBoardSta Key_SHIFT;	//16
	KeyBoardSta Key_CTRL;	//32
	
	KeyBoardSta Key_Q;	//64
	KeyBoardSta Key_E;  //128
	KeyBoardSta Key_R;	//256
	KeyBoardSta Key_F;	//512
	
	KeyBoardSta Key_G;	//1024
	KeyBoardSta Key_Z;	//2048
	KeyBoardSta Key_X;	//4096
	KeyBoardSta Key_C;	//8192
	KeyBoardSta Key_V;	//16384
	KeyBoardSta Key_B;	//32768

}CC_TypeDef;


#define RC_BUF_LEN    127

void MucRemoteInit( void );
void MucDmaCallbackRcHandle(RC_TypeDef* rc, uint8_t* buff, uint16_t size );
void RemoteTaskLoop( void );

extern unsigned char g_RcRawBuf[RC_BUF_LEN];
extern RC_TypeDef g_RcRawData;
extern volatile  float Mpu6500AngleData;
extern CC_TypeDef CC_Condition;
extern int Sta_To_Fllow;

#endif  //_MUC_REMOTE_H__

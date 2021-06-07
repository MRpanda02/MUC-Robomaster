/**
  ******************************************************************************
  * File Name          : muc_remote.c
  * Description        : 用于处理遥控器反馈的数据,并将处理后的数据保存到 CC_Condition 中
  ******************************************************************************
  *
  */

#include "muc_remote.h"
//#include "muc_hardware_uart.h"
#include <string.h>

//#include "muc_test.h"

CC_TypeDef CC_Condition={0};

uint16_t MouseLeftPressTime=0;
uint16_t MouseRightPressTime=0;
int16_t rcLostCheckCount = 0;

/**
    执行此函数后可以使用
*/
void MucRemoteInit( void )
{
#include "stm32f4xx.h"                  // Device header
#include "RTE_Components.h"             // Component selection
    memset(&CC_Condition,0,sizeof(CC_TypeDef));

    // 遥控器串口接口配置
    MucUartReceiveIT(&DBUS_Uart, g_RcRawBuf, RC_BUF_LEN);
    
}

unsigned char g_RcRawBuf[RC_BUF_LEN]={0};			//用于存储从串口接收到的数据
RC_TypeDef g_RcRawData={0};



void GetSwitchAction(void){
	
	static int8_t S1LastSta=0;
	static int8_t S2LastSta=0;
	
	if( S1LastSta==0)
	{
	S1LastSta = g_RcRawData.s1;
	S2LastSta = g_RcRawData.s2;
	}
		 
	CC_Condition.S1Sta = (SwitchState)g_RcRawData.s1;
	CC_Condition.S2Sta = (SwitchState)g_RcRawData.s2;

	if( g_RcRawData.s1 != S1LastSta )
	{
		switch( g_RcRawData.s1 )
		{
			case 1:
				CC_Condition.S1Action = SwitchAction_3TO1;
				break;
			case 2:
				CC_Condition.S1Action = SwitchAction_3TO2;
				break;
			case 3:
				if(S1LastSta == 2)
				{
				CC_Condition.S1Action = SwitchAction_2TO3;
				}
				else
				{
				CC_Condition.S1Action = SwitchAction_1TO3;
				}	
				break;
			default: break;	
		}
	}
	else
	{
		CC_Condition.S1Action = NoAction;
	}
		
	if( g_RcRawData.s2 != S2LastSta )
	{
		switch( g_RcRawData.s2 )
		{
			case 1:
				CC_Condition.S2Action = SwitchAction_3TO1;
				break;
			case 2: 
				CC_Condition.S2Action = SwitchAction_3TO2;
				break;
			case 3:
				if(S2LastSta == 2)
				{
				CC_Condition.S2Action = SwitchAction_2TO3;
				}
				else
				{
				CC_Condition.S2Action = SwitchAction_3TO1;
				}
				break;
			default: break;	
		}
	}
	else 
	{
		CC_Condition.S2Action = NoAction;
	}
	
	S1LastSta = g_RcRawData.s1;
	S2LastSta = g_RcRawData.s2;
}


void CheckKeyBoardStatus( void )
{
	CC_Condition.MouseLeftBt = (KeyBoardSta)(! g_RcRawData.mouse.press_left);
	CC_Condition.MouseRightBt = (KeyBoardSta)(! g_RcRawData.mouse.press_right);
	
	CC_Condition.Key_W = (KeyBoardSta)(!( g_RcRawData.key_code&Key_W_FLAG));
	CC_Condition.Key_S = (KeyBoardSta)(!( g_RcRawData.key_code&Key_S_FLAG));
	CC_Condition.Key_A = (KeyBoardSta)(!( g_RcRawData.key_code&Key_A_FLAG));
	CC_Condition.Key_D = (KeyBoardSta)(!( g_RcRawData.key_code&Key_D_FLAG));

	CC_Condition.Key_SHIFT 	= (KeyBoardSta)(!( g_RcRawData.key_code&Key_SHIFT_FLAG));
	CC_Condition.Key_CTRL 	= (KeyBoardSta)(!( g_RcRawData.key_code&Key_CTRL_FLAG));

	CC_Condition.Key_Q = (KeyBoardSta)(!( g_RcRawData.key_code&Key_Q_FLAG));
	CC_Condition.Key_E = (KeyBoardSta)(!( g_RcRawData.key_code&Key_E_FLAG));
	CC_Condition.Key_R = (KeyBoardSta)(!( g_RcRawData.key_code&Key_R_FLAG));
	CC_Condition.Key_F = (KeyBoardSta)(!( g_RcRawData.key_code&Key_F_FLAG));
	CC_Condition.Key_G = (KeyBoardSta)(!( g_RcRawData.key_code&Key_G_FLAG));
	CC_Condition.Key_Z = (KeyBoardSta)(!( g_RcRawData.key_code&Key_Z_FLAG));
	CC_Condition.Key_X = (KeyBoardSta)(!( g_RcRawData.key_code&Key_X_FLAG));
	CC_Condition.Key_C = (KeyBoardSta)(!( g_RcRawData.key_code&Key_C_FLAG));
	CC_Condition.Key_V = (KeyBoardSta)(!( g_RcRawData.key_code&Key_V_FLAG));
	CC_Condition.Key_B = (KeyBoardSta)(!( g_RcRawData.key_code&Key_B_FLAG));
}

void MouseDataSend(void)
{
	CC_Condition.x= g_RcRawData.mouse.x;
	CC_Condition.y= g_RcRawData.mouse.y;
	CC_Condition.z= g_RcRawData.mouse.z;
}

void RemoteDataSend(void)
{
	CC_Condition.ch1= g_RcRawData.ch1;
	CC_Condition.ch2= g_RcRawData.ch2;
	CC_Condition.ch3= g_RcRawData.ch3;
	CC_Condition.ch4= g_RcRawData.ch4;
}

void OfflineClear()
{
	g_RcRawData.ch1=0;
	g_RcRawData.ch2=0;
	g_RcRawData.ch3=0;
	g_RcRawData.ch4=0;
		
	g_RcRawData.key_code=0;
		
	g_RcRawData.mouse.press_left=0;
	g_RcRawData.mouse.press_right=0;
		
	g_RcRawData.mouse.x=0;
	g_RcRawData.mouse.y=0;
	g_RcRawData.mouse.z=0;
		
	g_RcRawData.s1=0;
	g_RcRawData.s2=0;
//		
//	CC_Condition.yaw_angle_ref=0;
//	CC_Condition.pitch_angle_ref=0;
//	CC_Condition.FrontBack=0;
//	CC_Condition.LeftRight=0;
}


void RemoteTaskLoop( void )
{
	if( rcLostCheckCount++>20 )
	{
	CC_Condition.rcSta = RC_OffLINE;
	rcLostCheckCount = 21;
	}
	else
	{
	CC_Condition.rcSta = RC_ONLINE;
	}

	   
    GetSwitchAction();
	CheckKeyBoardStatus();
	
	MouseDataSend();
	RemoteDataSend();
	
	switch(CC_Condition.rcSta)
	{
		case RC_OffLINE:
//			OfflineClear();	
			break;
		case  RC_ONLINE: 
			break;  
		default: break;	        
	}
}

void MucDmaCallbackRcHandle(RC_TypeDef* rc, uint8_t* buff, uint16_t size )
{

    // 收到遥控器数据
	rcLostCheckCount = 0;
    
	rc->ch1 = ((buff[0] | buff[1]<<8) & 0x07FF)-1024;
	
	rc->ch2 = ((buff[1]>>3 | buff[2]<<5 ) & 0x07FF)-1024;
	
	rc->ch3 = ((buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF)-1024;
	
	rc->ch4 = ((buff[4]>>1 | buff[5]<<7) & 0x07FF)-1024;		
	
	rc->s1 = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->s2 =  (buff[5] >> 4)& 0x0003 ;
	

		
	rc->mouse.x =  (int16_t)(buff[6] | (buff[7] << 8));
	rc->mouse.y =  (int16_t)(buff[8] | (buff[9] << 8));
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	//限制鼠标输入的最大值
	VAL_LIMIT( rc->mouse.x, -50, 50); 
	VAL_LIMIT( rc->mouse.y, -100, 100); 
	
	
	rc->mouse.press_left  = buff[12];
	rc->mouse.press_right = buff[13];
	
	rc->key_code = buff[14] | buff[15] << 8; 
 

}

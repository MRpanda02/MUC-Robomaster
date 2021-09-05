#include "muc_hardware_can.h"
#include "muc_gimbal.h"
#include "muc_chassis.h"
#include "muc_stir_wheel.h"
#include "muc_shoot_wheel.h"
#include "muc_Shoot_Judge_Control.h"
#include "muc_hardware_super_capacity.h"


//can filter must be initialized before use
void CanFilter_Init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef canfilter;
  
  //create memory to save the message, if not will raise error
  
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  //filtrate any ID you want here
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  
  canfilter.FilterActivation = ENABLE;
  canfilter.SlaveStartFilterBank = 14;
  
  //use different filter for can1&can2
  if(hcan == &hcan1)
  {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
    canfilter.FilterBank = 0;
    HAL_CAN_ConfigFilter(hcan, &canfilter);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  }
  if(hcan == &hcan2)
  {
    canfilter.FilterFIFOAssignment = CAN_FilterFIFO1;
    canfilter.FilterBank = 14;
    HAL_CAN_ConfigFilter(hcan, &canfilter);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  }
  
  HAL_CAN_Start(hcan);
  
}

void muc_can_wait( void )
{
    int32_t dtime_i=1000;
    while(dtime_i--);    
}

HAL_StatusTypeDef CAN_Send_Message(CAN_HandleTypeDef* _hcan, uint16_t _id, int16_t message1,int16_t message2,int16_t message3,int16_t message4)
{
    CAN_TxHeaderTypeDef  TxMessage;    
    uint8_t TxMessageData[8]={0};
    uint8_t trySendCounts=6;
    
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.StdId = _id;

    TxMessage.DLC = 8; 				
 
	TxMessageData[0] = (uint8_t)(message1>>8);
	TxMessageData[1] = (uint8_t)(message1);
	TxMessageData[2] = (uint8_t)(message2>>8);
	TxMessageData[3] = (uint8_t)(message2);
	TxMessageData[4] = (uint8_t)(message3>>8);
	TxMessageData[5] = (uint8_t)(message3);
	TxMessageData[6] = (uint8_t)(message4>>8);
	TxMessageData[7] = (uint8_t)(message4);
    /* 如果空闲发送邮箱空闲数目是0时,则延迟一会儿再发送，否则数据会发送无效 */
    while( (HAL_CAN_GetTxMailboxesFreeLevel(_hcan)==0) && (trySendCounts--))
    {
        muc_can_wait();
    }
	return HAL_CAN_AddTxMessage(_hcan,&TxMessage,TxMessageData,(uint32_t*)CAN_TX_MAILBOX0);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)  
{
    static uint8_t  rx1RawData[8];
    HAL_StatusTypeDef	HAL_RetVal;
    CAN_RxHeaderTypeDef  rxMessage;

    
    if(hcan ==&hcan1)
    {	
        HAL_RetVal=HAL_CAN_GetRxMessage(hcan,  CAN_RX_FIFO0, &rxMessage,  rx1RawData);
        if ( HAL_OK==HAL_RetVal)
        {                              			
            switch(rxMessage.StdId)
            {
                case 0x201:
                {
					ChassisDealDataFromCan(&CM1_Feedback,rx1RawData);
                    Shoot_Speed_Data_deal(&Shoot_Left_Feedback,rx1RawData);
                    break;	
                }
                case 0x202:
                {
					ChassisDealDataFromCan(&CM2_Feedback,rx1RawData);
                    Shoot_Speed_Data_deal(&Shoot_Right_Feedback,rx1RawData);
                    break;
                }
                case 0x203:
                {
					ChassisDealDataFromCan(&CM3_Feedback,rx1RawData);  
                    break;
                }
                case 0x204:
                {
                    ChassisDealDataFromCan(&CM4_Feedback,rx1RawData);
                    break;
                }
				
				    case 0x205:
                {
                    GimbalDealDataFromCan(&GMYawEncoder ,rx1RawData,GimbalYawMidOffset);
                    break;
                }
				
                case 0x206:
                {
					GimbalDealDataFromCan(&GMPitchEncoder ,rx1RawData,GimbalPitchMidOffset);
                    break;
                }
           
                case 0x207:
                {
					Position_Round_Data_deal(&Stir_position_Feedback,rx1RawData);
					Shooting_Speed_Data_deal(&Stir_speed_Feedback,rx1RawData);//暂时打算进行位置环控制
                    break;
                }

                default:
                    break;

            }
        }
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)  
{
    static uint8_t  rx2RawData[8];
    HAL_StatusTypeDef	HAL_RetVal;
    CAN_RxHeaderTypeDef  rxMessage;
    
    if(hcan == &hcan2)
    {	
        HAL_RetVal=HAL_CAN_GetRxMessage(&hcan2,  CAN_RX_FIFO1, &rxMessage,  rx2RawData);
        if ( HAL_OK==HAL_RetVal)
        {                              			
            switch(rxMessage.StdId)
            {
                case 0x201:
                {
                    ChassisDealDataFromCan(&CM1_Feedback,rx2RawData);
                    break;	
                }
                case 0x202:
                {
                    ChassisDealDataFromCan(&CM2_Feedback,rx2RawData);
                    break;
                }
                case 0x203:
                {
                     ChassisDealDataFromCan(&CM3_Feedback,rx2RawData);
                    break;
                }
                case 0x204:
                {
                   ChassisDealDataFromCan(&CM4_Feedback,rx2RawData);
                    break;
                }
                  case 0x320:
                {
                    break;
                }
				  case 0x141:
				{
		           Deal_Data_From_Chassis1( &shoot_control_consulation ,rx2RawData);	
				   break;
				}
				
				  case 0x241:
				{
		           Deal_Data_From_Chassis2( &shoot_control_consulation ,rx2RawData);		
				   break;
				}
					case 0x140:
				{	
				   Deal_Data_ForShift( ShiftToChassis, rx2RawData);
				   break;
				}
	
                default:
                    break;

            }
        }
    }
}

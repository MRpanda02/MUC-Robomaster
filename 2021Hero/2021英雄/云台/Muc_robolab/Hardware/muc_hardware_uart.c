#include "muc_hardware_uart.h"
#include "muc_remote.h"
#include "muc_referee_system.h"
#include "muc_auto_shoot.h"
#include "muc_visual_tasks.h"

// 支持Printf串口输出
int fputc(int ch,FILE *f)
{
    uint8_t temp[1]={ch};
    HAL_UART_Transmit(&huart7,temp,1,2);
    return ch; 
}

/**
  * @brief  因为串口开启没有中断的DMA传输，为了减少中断次数为其他中断空出资源。
  *         代替HAL库的函数(此处在main函数中调用)
  * @param  hdma: 指向DMA_HandleTypeDef结构体的指针，这个结构体包含了DMA流的配置信息.  
  * @retval HAL status
  */
HAL_StatusTypeDef MUC_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  uint32_t *tmp;
  uint32_t tmp1 = 0;
  
  tmp1 = huart->RxState;
  if((tmp1 == HAL_UART_STATE_READY) || (tmp1 == HAL_UART_STATE_BUSY_TX))
  {
    if((pData == NULL ) || (Size == 0)) 
    {
      return HAL_ERROR;
    }
    
    /* Process Locked */
    __HAL_LOCK(huart);
    
    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    /* Check if a transmit process is ongoing or not */
    if(huart->RxState == HAL_UART_STATE_BUSY_TX)
    {
      huart->RxState = HAL_UART_STATE_BUSY_TX_RX;
    }
    else
    {
      huart->RxState = HAL_UART_STATE_BUSY_RX;
    }
    
    /* Enable the DMA Stream */
    tmp = (uint32_t*)&pData;
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t*)tmp, Size);
    
    /* Enable the DMA transfer for the receiver request by setting the DMAR bit 
    in the UART CR3 register */
    huart->Instance->CR3 |= USART_CR3_DMAR;
    
    /* Process Unlocked */
    __HAL_UNLOCK(huart);
    
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY; 
  }
}


// 打开串口接收中断
HAL_StatusTypeDef MucUartReceiveIT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	
    MUC_UART_Receive_DMA(huart,pData,Size);

	return HAL_OK;
}

void UART_IdleRxCallback( UART_HandleTypeDef *huart, uint16_t len )
{
	if(huart==&DBUS_Uart){		
		MucDmaCallbackRcHandle(&g_RcRawData, g_RcRawBuf, len );
	}
	else if(huart==&MiniPC_Uart){
		Dma_Callback_MS_Handle(&Version_Shooting,g_MiniPCRxBuf);
	}

//	else if(huart==&Referee_Uart){
//		MucCallbackRefereeSystemHandle(&judge_unpack_obj, g_judgeRxDataBuf);
//	}


}

// 在中断函数中被调用，可以搜索全局查找
void MucUsartIdleHanlder(UART_HandleTypeDef *huart,uint16_t Size)
{
    uint32_t DMA_FLAGS;
    UART_IdleRxCallback(huart,Size-huart->hdmarx->Instance->NDTR);
    if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart); 
        DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);
        __HAL_DMA_DISABLE(huart->hdmarx);
        __HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_FLAGS);
        __HAL_DMA_SET_COUNTER(huart->hdmarx,Size);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}

//以下三个函数看起来没什么用，也没被调用，但是是读取裁判系统必需的，原因不明，不得删改
/**
  * @brief  Returns the current memory target used by double buffer transfer.
  * @param  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @retval The memory target number: 0 for Memory0 or 1 for Memory1. 
  */
uint8_t dma_current_memory_target(DMA_Stream_TypeDef *dma_stream)
{
  uint8_t tmp = 0;

  /* Get the current memory target */
  if ((dma_stream->CR & DMA_SxCR_CT) != 0)
  {
    /* Current memory buffer used is Memory 1 */
    tmp = 1;
  }
  else
  {
    /* Current memory buffer used is Memory 0 */
    tmp = 0;
  }
  return tmp;
}
/**
  * @brief  Returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @retval The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->NDTR));
}

void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt)
{
  *mem_id     = dma_current_memory_target(dma_stream);
  *remain_cnt = dma_current_data_counter(dma_stream);
}



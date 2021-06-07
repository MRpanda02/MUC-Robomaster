#include "muc_referee_system.h"
#include "muc_hardware_uart.h"
#include "string.h"



uint8_t g_judgeRxDataBuf[Judge_Buf_LEN];
uint8_t g_judgeTxDataBuf[100];

unpack_data_t judge_unpack_obj;

receive_judge_t g_refereeSystemReceMesg;


void MucRefereeSystemInit( void )
{
   
    judge_unpack_obj.p_header = (frame_header_t *)judge_unpack_obj.protocol_packet;
    judge_unpack_obj.index = 0;
    judge_unpack_obj.data_len = 0;
    judge_unpack_obj.unpack_step = STEP_HEADER_SOF;

    // 裁判系统串口接口配置
    MucUartReceiveIT(&Referee_Uart, g_judgeRxDataBuf, Judge_Buf_LEN);
}


void RefereeSystemDataHandler(uint8_t *p_frame)
{
	frame_header_t *p_header = (frame_header_t*)p_frame;
	memcpy(p_header, p_frame, HEADER_LEN);
	uint16_t data_length = p_header->data_length;
	uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
	uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;
	
//  uint8_t  invalid_cmd = 0;//ID标志位判断无效ID
	switch (cmd_id)
	{
		case ID_game_state:	
		{
			memcpy(&g_refereeSystemReceMesg.game_state, data_addr, data_length);
		//    printf("当前阶段剩余时间=%ds\n",judge_rece_mesg.game_state.stage_remain_time);
		}break;

		case ID_game_result:
		{
			memcpy(&g_refereeSystemReceMesg.game_result, data_addr, data_length);
			//	printf("比赛结果：%d\n",judge_rece_mesg.game_result.winner);
		}break;

		case ID_game_robot_survivors:
		{
		memcpy(&g_refereeSystemReceMesg.game_robot_survivors, data_addr, data_length);
		//printf("机器人存活数据%d\n",judge_rece_mesg.game_robot_survivors.robot_legion);
		}break;

		case ID_event_data:
		{
		memcpy(&g_refereeSystemReceMesg.event_data, data_addr, data_length);
		//	printf("场地事件数据%d",judge_rece_mesg.event_data.event_type);
		}break;

		case ID_supply_projectile_action:
		{
		memcpy(&g_refereeSystemReceMesg.supply_projectile_action, data_addr, data_length);
		//	printf("补给站动作%d\n",judge_rece_mesg.supply_projectile_action.supply_robot_id);
		}break;

		case ID_supply_projectile_booking:
		{
		memcpy(&g_refereeSystemReceMesg.supply_projectile_booking, data_addr, data_length);
		//		printf("补给站预约%d\n",judge_rece_mesg.supply_projectile_booking.supply_robot_id);
		}break;

		case ID_game_robot_state:
		{
		memcpy(&g_refereeSystemReceMesg.game_robot_state, data_addr, data_length);
		//	printf("robotHP:%d\n",judge_rece_mesg.game_robot_state.remain_HP );
		//	printf("robot level:%d\n",judge_rece_mesg.game_robot_state.robot_level);
		}break;

		case ID_power_heat_data:
		{
		memcpy(&g_refereeSystemReceMesg.power_heat_data, data_addr, data_length);
		//	printf("枪口热量:%d\n",judge_rece_mesg.power_heat_data.chassis_shooter_heat0 );
		}break;

		case ID_game_robot_pos:
		{
		memcpy(&g_refereeSystemReceMesg.game_robot_pos, data_addr, data_length);
		//	printf("机器人位置:%f\n",judge_rece_mesg.game_robot_pos.yaw);
		}break;

		case ID_buff_musk:
		{
		memcpy(&g_refereeSystemReceMesg.buff_musk, data_addr, data_length);
		//	printf("机器人增益：%d",judge_rece_mesg.buff_musk.power_rune_buff);
		}break;

		case ID_aerial_robot_energy:
		{
		memcpy(&g_refereeSystemReceMesg.aerial_robot_energy, data_addr, data_length);
		//	printf("积累的能量点:%d\n",judge_rece_mesg.aerial_robot_energy.energy_point);
		}break;

		case ID_robot_hurt:
		{
		memcpy(&g_refereeSystemReceMesg.robot_hurt, data_addr, data_length);
		//	printf("armor is:%d\n",judge_rece_mesg.robot_hurt.armor_id );
		//		printf ("type is:%d\n",judge_rece_mesg.robot_hurt.hurt_type );
		}break;

		case ID_shoot_data:
		{
		memcpy(&g_refereeSystemReceMesg.shoot_data, data_addr, data_length);
		//		printf("射频：%ds\n",judge_rece_mesg.shoot_data.bullet_freq);
		}break;

		case ID_student_interative_header_data:
		{
		memcpy(&g_refereeSystemReceMesg.student_interative, data_addr, data_length);
		//		printf("transmit\n");
		}break;



		default:
		{
			
		}break;
		//      invalid_cmd = 1;//无效Id
	}
}

void MucCallbackRefereeSystemHandle(unpack_data_t *p_obj, uint8_t* buff)
{
	uint8_t byte = 0;
	uint16_t i=0;
	uint8_t stop = 0;

	while(!stop)
	{
		byte=buff[i++];
		if(i>=Judge_Buf_LEN)
		{
		p_obj->unpack_step = STEP_HEADER_SOF;
		p_obj->index = 0;
		stop=1;
	    }

		switch(p_obj->unpack_step)
			{
				case STEP_HEADER_SOF:
				{
					if(byte == DN_REG_ID)
					{ 
						p_obj->unpack_step = STEP_LENGTH_LOW;
						p_obj->protocol_packet[p_obj->index++] = byte;
					}else
					{
						p_obj->index = 0;
						stop=1;
					}
				}break;

				case STEP_LENGTH_LOW:
				{ 
					p_obj->data_len = byte;
					p_obj->protocol_packet[p_obj->index++] = byte;
					p_obj->unpack_step = STEP_LENGTH_HIGH;
				}break;

				case STEP_LENGTH_HIGH:
				{ 
					p_obj->data_len |= (byte << 8);
					p_obj->protocol_packet[p_obj->index++] = byte;

					if(p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
					{
						p_obj->unpack_step = STEP_FRAME_SEQ;
					}else
					{
						p_obj->unpack_step = STEP_HEADER_SOF;
						p_obj->index = 0;
					}
				}break;

				case STEP_FRAME_SEQ:
				{ 
					p_obj->protocol_packet[p_obj->index++] = byte;
					p_obj->unpack_step = STEP_HEADER_CRC8;
				}break;

				case STEP_HEADER_CRC8:
				{ 
					p_obj->protocol_packet[p_obj->index++] = byte;

					if (p_obj->index == HEADER_LEN)
					{ 
						if ( verify_crc8_check_sum(p_obj->protocol_packet, HEADER_LEN) )
						{
							p_obj->unpack_step = STEP_DATA_CRC16;
						}
						else{ 
							p_obj->unpack_step = STEP_HEADER_SOF;
							p_obj->index = 0;
						}
					}
				}break;  

				case STEP_DATA_CRC16:
				{
					if (p_obj->index < (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
					{
						p_obj->protocol_packet[p_obj->index++] = byte;  
					}
					
					if (p_obj->index >= (HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN))
					{
						p_obj->unpack_step = STEP_HEADER_SOF;
						p_obj->index = 0;

						if ( verify_crc16_check_sum(p_obj->protocol_packet, HEADER_LEN + CMD_LEN + p_obj->data_len + CRC_LEN) )
						{
							RefereeSystemDataHandler(p_obj->protocol_packet);
						}
					}
				}break;

				default:
				{
					p_obj->unpack_step = STEP_HEADER_SOF;
					p_obj->index = 0;
				}break;
			}
	}
}

uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
    memset(tx_buf, 0, 100);
    static uint8_t seq;

    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
    frame_header_t *p_header = (frame_header_t*)tx_buf;

    p_header->sof          = sof;
    p_header->data_length  = len;
    p_header->seq          = 0;
    p_header->seq          = seq++;

    memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
    append_crc8_check_sum(tx_buf, HEADER_LEN);
    memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
    append_crc16_check_sum(tx_buf, frame_length);

    return tx_buf;
}

void data_upload_handler(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
//    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN; 
    protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf); 
    HAL_UART_Transmit(&Referee_Uart, g_judgeTxDataBuf, 100, 100);
}


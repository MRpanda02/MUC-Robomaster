#ifndef _MUC_REFEREE_SYSTEM_H__
#define _MUC_REFEREE_SYSTEM_H__
#include "main.h"
#include "protocol.h"

#define Judge_Buf_LEN   500
#define Interactive_Data_Len 100  //客户端自定义数据长度<113

/** 
  * @brief  judgement data command id
  */
typedef enum
{
    ID_game_state = 0x0001,//比赛状态信息   
    ID_game_result, //比赛结果数据（0x0002）
    ID_game_robot_survivors, //比赛机器人存活数据(0×0003)
    ID_event_data=0x0101, //场地事件数据，事件改变后发送 (0x0101)
    ID_supply_projectile_action, //场地补给站动作标识数据，动作改变后发送 (0x0102)
    ID_supply_projectile_booking, //场地补给站预约子弹数据，由参赛队发送，上限 10Hz。（ RM 对抗赛尚未开放）(0x0103)
    ID_game_robot_state=0x0201, //机器人状态数据，10Hz 周期发送 (0x0201)
    ID_power_heat_data, //实时功率热量数据，50Hz 周期发送 (0x0202)
    ID_game_robot_pos,  //机器人位置数据，10Hz 发送 (0x0203)
    ID_buff_musk,       //机器人增益数据,增益状态改变后发送 (0x0204)
    ID_aerial_robot_energy, //空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送 (0x0205)
    ID_robot_hurt,      //伤害状态数据，伤害发生后发送 (0x0206)
    ID_shoot_data,      //实时射击数据，子弹发射后发送 (0x0207)
    ID_student_interative_header_data=0x0301,//机器人间交互数据，发送方触发发送，上限 10Hz (0x0301)

    wrong = 0x1301 //枚举无效，只是为了使该枚举大小为2字节
} judge_data_id_e;//命令码

/*********************/
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;//对数据依次校验

typedef struct
{
  frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;
/***************************/
//以上结构体为头帧校验必须用到的，涉及底层

//以下为2019裁判系统数据
typedef __packed struct
{
	uint8_t game_type:4;//0-3bit 比赛类型
/*1：RM 对抗赛；2：单项赛；3：RM ICRA*/ 
	
	uint8_t game_progress:4;//4-7bit 当前比赛阶段
/* 0：未开始比赛；1：准备阶段；2：自检阶段；3：5s 倒计时；4：对战中；5：比赛结算中*/
	
	uint16_t stage_remain_time;//当前阶段剩余时间，单位 s
}ext_game_state_t; //比赛状态信息（0x0001）

typedef __packed struct
{
	uint8_t winner;//0 平局 1 红方胜利 2 蓝方胜利 
}ext_game_result_t;//比赛结果数据（0x0002）

typedef __packed struct
{
	uint16_t robot_legion;
/*bit 0：红方英雄机器人； 
	bit 1：红方工程机器人； 
	bit 2：红方步兵机器人 1； 
	bit 3：红方步兵机器人 2； 
	bit 4：红方步兵机器人 3； 
	bit 5：红方空中机器人； 
	bit 6：红方哨兵机器人； 
	bit 7：保留 
	bit 8：蓝方英雄机器人； 
	bit 9：蓝方工程机器人； 
	bit 10：蓝方步兵机器人 1； 
	bit 11：蓝方步兵机器人 2； 
	bit 12：蓝方步兵机器人 3； 
	bit 13：蓝方空中机器人； 
	bit 14：蓝方哨兵机器人； 
	bit 15：保留 
对应的 bit 数值置 1 代表机器人存活，数值置 0 代表机器人死亡或者未上场*/
}ext_game_robot_survivors_t;//机器人存活数据 (0×0003)

typedef __packed struct
{
	uint32_t event_type;
/*bit 0-1：己方停机坪占领状态  
	0为无机器人占领； 
	1为空中机器人已占领但未停桨； 
	2为空中机器人已占领并停桨 
	bit 2：己方补给站 1号补血点占领状态 1为已占领； 
	bit 3：己方补给站 2号补血点占领状态 1为已占领； 
	bit 4：己方补给站 3号补血点占领状态 1为已占领； 
	bit 5-6：己方大能量机关状态： 
	0为打击点未占领且大能量机关未激活； 
	1为打击点占领且大能量机关未激活； 
	2为大能量机关已激活； 
	3为大能量机关已激活且打击点被占领； 
	bit 7：己方关口占领状态 1为已占领； 
	bit 8：己方碉堡占领状态 1为已占领； 
	bit 9：己方资源岛占领状态 1为已占领； 
	bit 10-11：己方基地防御状态： 
	2 为基地 100%防御； 
	1为基地有哨兵防御； 
	0为基地无防御； 
	bit 12-13：ICRA 红方防御加成 
	0：防御加成未激活； 
	1：防御加成 5s触发激活中； 
	2：防御加成已激活 
	bit 14-15：ICRA 蓝方防御加成 
	0：防御加成未激活； 
	1：防御加成 5s触发激活中； 
	2：防御加成已激活 
	其余保留 */
}ext_event_data_t; //场地事件数据(0×0101)

typedef __packed struct
{
	uint8_t supply_projectile_id;//补给站口 ID：1：1 号补给口；2 号补给口 
	uint8_t supply_robot_id;//预约机器人 ID：0 为当前无预约，1 为红方英雄预约，以此类推其他机器人 ID 号预约
	uint8_t supply_projectile_step;//子弹口开闭状态：0 为关闭，1 为子弹准备中，2 为子弹下落 
	uint8_t supply_projectile_num;//补弹数量： 50：50 颗子弹； 100：100 颗子弹； 150：150 颗子弹； 200：200 颗子弹
}ext_supply_projectile_action_t; //补给站动作标识(0×0102)

typedef __packed struct//RM对抗赛尚未开放
{
	uint8_t supply_project_id;
/*预约补给站口 ID： 
	0：空闲补给口，依照 1，2 顺序查询补给空
	闲情况； 
	1：1 号补给口； 
	2：2 号补给口*/
	uint8_t supply_robot_id;//补弹机器人ID：
/*1为红方英雄机器人补弹，
	2为红方工程机器人补弹， 
	3/4/5 为红方步兵机器人补弹，
	11 为蓝方英雄机器人补弹，
	12 为蓝方 工程机器人补弹，
	13/14/15为蓝方步兵机器人补弹 */
	uint8_t supply_project_num;
/*预约子弹数目：  
0-50 为预约 50 颗子弹，  
51-100 为预约 100 颗子弹，
101-150 为预约150 颗子弹， 
151-255 为预约 200 颗子弹。（上限 200 颗子弹）*/ 
 
}ext_supply_projectile_booking_t; //补给站预约子弹(0×0103)

typedef __packed struct
{
	uint8_t  robot_id;
/*机器人 ID： 
	1：红方英雄机器人； 
	2： 红方工程机器人； 
	3/4/5，红方步兵机器人； 
	6，红方空中机器人； 
	7，红方哨兵机器人； 
	11，蓝方英雄机器人； 
	12，蓝方工程机器人； 
	13/14/15，蓝方步兵机器人； 
	16，蓝方空中机器人； 
	17，蓝方哨兵机器人。 */
	uint8_t  robot_level;//机器人等级：1：一级；2：二级；3：三级。 
	uint16_t remain_HP;//机器人剩余血量 
	uint16_t max_HP;//机器人满血量 
	uint16_t shooter_heat0_cooling_rate;//机器人 17mm 子弹热量冷却速度 单位 /s 
	uint16_t shooter_heat0_cooling_limit;//机器人 17mm 子弹热量上限 
	uint16_t shooter_heat1_cooling_rate;//机器人 42mm 子弹热量冷却速度 单位 /s 
	uint16_t shooter_heat1_cooling_limit;//机器人 42mm 子弹热量上限 
	uint8_t  mains_power_gimbal_output:1;//gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出
	uint8_t  mains_power_chassis_output:1;//chassis 口输出：1 为有 24V 输出，0 为无 24v 输出
	uint8_t  mains_power_shooter_output:1;//shooter 口输出：1 为有 24V 输出，0 为无 24v 输出
}ext_game_robot_state_t; //比赛机器人状态(0×0201)

typedef __packed struct
{
	uint16_t chassis_volt;//底盘输出电压 单位 毫伏 
	uint16_t chassis_current;//底盘输出电流 单位 毫安 
	float    chassis_power;//底盘输出功率 单位 W 瓦 
	uint16_t chassis_power_buffer;//底盘功率缓冲 单位 J 焦耳 
	uint16_t chassis_shooter_heat0;//17mm 枪口热量 
	uint16_t chassis_shooter_heat1;//42mm 枪口热量 
}ext_power_heat_data_t; //实时功率热量数据(0×0202)

typedef __packed struct
{
	//单位m
	float x;
	float y;
	float z;
	float yaw;//位置枪口，单位度 
}ext_game_robot_pos_t; //机器人位置(0×0203)

typedef __packed struct
{
	uint8_t power_rune_buff;
/*bit 0：机器人血量补血状态 
	bit 1：枪口热量冷却加速 
	bit 2：机器人防御加成 
	bit 3：机器人攻击加成 
	其他 bit 保留 */
}ext_buff_musk_t; //机器人增益(0×0204)

typedef __packed struct
{
	uint8_t energy_point;//积累的能量点 
	uint8_t attack_time;//可攻击时间 单位 s。50s 递减至 0 
}aerial_robot_energy_t; //空中机器人能量状态(0×0205)

typedef __packed struct
{
	uint8_t armor_id:4;
//bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0。 
	uint8_t hurt_type:4;
/*bit 4-7：血量变化类型 
	0x0 装甲伤害扣血； 
	0x1 模块掉线扣血； 
	0x2 超枪口热量扣血； 
	0x3 超底盘功率扣血。 */
}ext_robot_hurt_t; //伤害状态(0×0206)

typedef __packed struct
{
	uint8_t bullet_type;//子弹类型: 1：17mm 弹丸 2：42mm 弹丸 
	uint8_t bullet_freq;//子弹射频 单位 Hz 
	float   bullet_speed;//子弹射速 单位 m/s 
}ext_shoot_data_t; //实时射击信息(0×0207)

//机器人间交互数据

typedef __packed struct
{
	float   data1;//自定义浮点数据 1
	float   data2;//自定义浮点数据 2
	float   data3;//自定义浮点数据 3
	uint8_t masks;//bit 0-5：分别控制客户端自定义数据显示面板上的六个指示 灯，值为 1时显示绿色，值为 0是显示红色。 Bit 6-7：保留 
}client_custom_data_t;//客户端 (客户端自定义数据：cmd_id:0x0301。内容ID:0xD180)

typedef __packed struct
{
	uint8_t data[Interactive_Data_Len];
}robot_interactive_data_t;//学生机器人间通信(客户端自定义数据：cmd_id:0x0301。内容ID:0x0201~0x02FF)

typedef __packed struct
{
	__packed struct
	{
	uint16_t data_cmdid;//内容ID
	uint16_t send_ID;//需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需 要校验红 1 
	uint16_t receiver_ID;//需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的 ID 
	}ext_student_interative_header_data_t;//数据段头结构
/*ID说明：1.内容ID：０ｘＤ１８０客户端自定义数据
	０ｘ０２００～０ｘ０２ｆｆ己方机器人间通信
	２.发送或接受者的ID：
1. 机器人 ID：1，英雄(红)；2，工程(红)；3/4/5，步兵(红)；6，空中(红)；7，哨兵(红)；11，英雄(蓝)； 12，工程(蓝)；13/14/15，步兵(蓝)；16，空中(蓝)；17，哨兵(蓝)。 
2. 客户端 ID：0x0101 为英雄操作手客户端(红)；0x0102， 工程操作手客户端((红)； 0x0103/0x0104/0x0105，步兵操作手客户端(红)；0x0106，空中操作手客户端((红)； 0x0111，英 雄操作手客户端(蓝)；0x0112，工程操作手客户端(蓝)；0x0113/0x0114/0x0115，操作手客户端步 兵(蓝)；0x0116，空中操作手客户端(蓝)
*/	
	__packed union
  {	
		client_custom_data_t       client_custom_data;//内容ID:(0xD108) 客户端
		robot_interactive_data_t   robot_interactive_data;//内容ID:(0x0201~0x02FF) 学生机器人间通信
	}ReceiveData;//交互数据内容
}ext_student_interative_t;//交互数据接收信息(0x0301)

/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
    ext_game_state_t                game_state;//(0x0001)比赛状态信息
    ext_game_result_t               game_result;//(0x0002)比赛结果数据
    ext_game_robot_survivors_t      game_robot_survivors;//(0x0003)机器人存活数据
    ext_event_data_t                event_data;//(0×0101)场地事件数据
    ext_supply_projectile_action_t  supply_projectile_action; //(0×0102)补给站动作标识
    ext_supply_projectile_booking_t supply_projectile_booking;//(0×0103)补给站预约子弹
    ext_game_robot_state_t          game_robot_state;//(0×0201)比赛机器人状态
    ext_power_heat_data_t           power_heat_data;//(0×0202)实时功率热量数据
    ext_game_robot_pos_t            game_robot_pos; //(0×0203)机器人位置
    ext_buff_musk_t                 buff_musk; //(0×0204)机器人增益
    aerial_robot_energy_t           aerial_robot_energy; //(0×0205)空中机器人能量状态
    ext_robot_hurt_t                robot_hurt; //(0×0206)伤害状态
    ext_shoot_data_t                shoot_data; //(0×0207)实时射击信息
    ext_student_interative_t        student_interative;//(0x0301)交互数据接收信息
} receive_judge_t;//裁判系统串口接受的数据结构体


void MucRefereeSystemInit( void );
void MucCallbackRefereeSystemHandle(unpack_data_t *p_obj, uint8_t* buff);
void data_upload_handler(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);

extern uint8_t g_judgeRxDataBuf[Judge_Buf_LEN];
extern unpack_data_t judge_unpack_obj;
extern receive_judge_t g_refereeSystemReceMesg;
extern uint8_t g_judgeTxDataBuf[100];
#endif

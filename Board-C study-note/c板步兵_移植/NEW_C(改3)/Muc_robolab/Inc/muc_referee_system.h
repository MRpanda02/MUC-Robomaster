#ifndef _MUC_REFEREE_SYSTEM_H__
#define _MUC_REFEREE_SYSTEM_H__
#include "main.h"
#include "protocol.h"

#define Judge_Buf_LEN   500
#define Interactive_Data_Len 100  //�ͻ����Զ������ݳ���<113

/** 
  * @brief  judgement data command id
  */
typedef enum
{
    ID_game_state = 0x0001,//����״̬��Ϣ   
    ID_game_result, //����������ݣ�0x0002��
    ID_game_robot_survivors, //���������˴������(0��0003)
    ID_event_data=0x0101, //�����¼����ݣ��¼��ı���� (0x0101)
    ID_supply_projectile_action, //���ز���վ������ʶ���ݣ������ı���� (0x0102)
    ID_supply_projectile_booking, //���ز���վԤԼ�ӵ����ݣ��ɲ����ӷ��ͣ����� 10Hz���� RM �Կ�����δ���ţ�(0x0103)
    ID_game_robot_state=0x0201, //������״̬���ݣ�10Hz ���ڷ��� (0x0201)
    ID_power_heat_data, //ʵʱ�����������ݣ�50Hz ���ڷ��� (0x0202)
    ID_game_robot_pos,  //������λ�����ݣ�10Hz ���� (0x0203)
    ID_buff_musk,       //��������������,����״̬�ı���� (0x0204)
    ID_aerial_robot_energy, //���л���������״̬���ݣ�10Hz ���ڷ��ͣ�ֻ�п��л��������ط��� (0x0205)
    ID_robot_hurt,      //�˺�״̬���ݣ��˺��������� (0x0206)
    ID_shoot_data,      //ʵʱ������ݣ��ӵ�������� (0x0207)
    ID_student_interative_header_data=0x0301,//�����˼佻�����ݣ����ͷ��������ͣ����� 10Hz (0x0301)

    wrong = 0x1301 //ö����Ч��ֻ��Ϊ��ʹ��ö�ٴ�СΪ2�ֽ�
} judge_data_id_e;//������

/*********************/
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;//����������У��

typedef struct
{
  frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;
/***************************/
//���Ͻṹ��Ϊͷ֡У������õ��ģ��漰�ײ�

//����Ϊ2019����ϵͳ����
typedef __packed struct
{
	uint8_t game_type:4;//0-3bit ��������
/*1��RM �Կ�����2����������3��RM ICRA*/ 
	
	uint8_t game_progress:4;//4-7bit ��ǰ�����׶�
/* 0��δ��ʼ������1��׼���׶Σ�2���Լ�׶Σ�3��5s ����ʱ��4����ս�У�5������������*/
	
	uint16_t stage_remain_time;//��ǰ�׶�ʣ��ʱ�䣬��λ s
}ext_game_state_t; //����״̬��Ϣ��0x0001��

typedef __packed struct
{
	uint8_t winner;//0 ƽ�� 1 �췽ʤ�� 2 ����ʤ�� 
}ext_game_result_t;//����������ݣ�0x0002��

typedef __packed struct
{
	uint16_t robot_legion;
/*bit 0���췽Ӣ�ۻ����ˣ� 
	bit 1���췽���̻����ˣ� 
	bit 2���췽���������� 1�� 
	bit 3���췽���������� 2�� 
	bit 4���췽���������� 3�� 
	bit 5���췽���л����ˣ� 
	bit 6���췽�ڱ������ˣ� 
	bit 7������ 
	bit 8������Ӣ�ۻ����ˣ� 
	bit 9���������̻����ˣ� 
	bit 10���������������� 1�� 
	bit 11���������������� 2�� 
	bit 12���������������� 3�� 
	bit 13���������л����ˣ� 
	bit 14�������ڱ������ˣ� 
	bit 15������ 
��Ӧ�� bit ��ֵ�� 1 ��������˴���ֵ�� 0 �����������������δ�ϳ�*/
}ext_game_robot_survivors_t;//�����˴������ (0��0003)

typedef __packed struct
{
	uint32_t event_type;
/*bit 0-1������ͣ��ƺռ��״̬  
	0Ϊ�޻�����ռ�죻 
	1Ϊ���л�������ռ�쵫δͣ���� 
	2Ϊ���л�������ռ�첢ͣ�� 
	bit 2����������վ 1�Ų�Ѫ��ռ��״̬ 1Ϊ��ռ�죻 
	bit 3����������վ 2�Ų�Ѫ��ռ��״̬ 1Ϊ��ռ�죻 
	bit 4����������վ 3�Ų�Ѫ��ռ��״̬ 1Ϊ��ռ�죻 
	bit 5-6����������������״̬�� 
	0Ϊ�����δռ���Ҵ���������δ��� 
	1Ϊ�����ռ���Ҵ���������δ��� 
	2Ϊ�����������Ѽ�� 
	3Ϊ�����������Ѽ����Ҵ���㱻ռ�죻 
	bit 7�������ؿ�ռ��״̬ 1Ϊ��ռ�죻 
	bit 8�������ﱤռ��״̬ 1Ϊ��ռ�죻 
	bit 9��������Դ��ռ��״̬ 1Ϊ��ռ�죻 
	bit 10-11���������ط���״̬�� 
	2 Ϊ���� 100%������ 
	1Ϊ�������ڱ������� 
	0Ϊ�����޷����� 
	bit 12-13��ICRA �췽�����ӳ� 
	0�������ӳ�δ��� 
	1�������ӳ� 5s���������У� 
	2�������ӳ��Ѽ��� 
	bit 14-15��ICRA ���������ӳ� 
	0�������ӳ�δ��� 
	1�������ӳ� 5s���������У� 
	2�������ӳ��Ѽ��� 
	���ౣ�� */
}ext_event_data_t; //�����¼�����(0��0101)

typedef __packed struct
{
	uint8_t supply_projectile_id;//����վ�� ID��1��1 �Ų����ڣ�2 �Ų����� 
	uint8_t supply_robot_id;//ԤԼ������ ID��0 Ϊ��ǰ��ԤԼ��1 Ϊ�췽Ӣ��ԤԼ���Դ��������������� ID ��ԤԼ
	uint8_t supply_projectile_step;//�ӵ��ڿ���״̬��0 Ϊ�رգ�1 Ϊ�ӵ�׼���У�2 Ϊ�ӵ����� 
	uint8_t supply_projectile_num;//���������� 50��50 ���ӵ��� 100��100 ���ӵ��� 150��150 ���ӵ��� 200��200 ���ӵ�
}ext_supply_projectile_action_t; //����վ������ʶ(0��0102)

typedef __packed struct//RM�Կ�����δ����
{
	uint8_t supply_project_id;
/*ԤԼ����վ�� ID�� 
	0�����в����ڣ����� 1��2 ˳���ѯ������
	������� 
	1��1 �Ų����ڣ� 
	2��2 �Ų�����*/
	uint8_t supply_robot_id;//����������ID��
/*1Ϊ�췽Ӣ�ۻ����˲�����
	2Ϊ�췽���̻����˲����� 
	3/4/5 Ϊ�췽���������˲�����
	11 Ϊ����Ӣ�ۻ����˲�����
	12 Ϊ���� ���̻����˲�����
	13/14/15Ϊ�������������˲��� */
	uint8_t supply_project_num;
/*ԤԼ�ӵ���Ŀ��  
0-50 ΪԤԼ 50 ���ӵ���  
51-100 ΪԤԼ 100 ���ӵ���
101-150 ΪԤԼ150 ���ӵ��� 
151-255 ΪԤԼ 200 ���ӵ��������� 200 ���ӵ���*/ 
 
}ext_supply_projectile_booking_t; //����վԤԼ�ӵ�(0��0103)

typedef __packed struct
{
	uint8_t  robot_id;
/*������ ID�� 
	1���췽Ӣ�ۻ����ˣ� 
	2�� �췽���̻����ˣ� 
	3/4/5���췽���������ˣ� 
	6���췽���л����ˣ� 
	7���췽�ڱ������ˣ� 
	11������Ӣ�ۻ����ˣ� 
	12���������̻����ˣ� 
	13/14/15���������������ˣ� 
	16���������л����ˣ� 
	17�������ڱ������ˡ� */
	uint8_t  robot_level;//�����˵ȼ���1��һ����2��������3�������� 
	uint16_t remain_HP;//������ʣ��Ѫ�� 
	uint16_t max_HP;//��������Ѫ�� 
	uint16_t shooter_heat0_cooling_rate;//������ 17mm �ӵ�������ȴ�ٶ� ��λ /s 
	uint16_t shooter_heat0_cooling_limit;//������ 17mm �ӵ��������� 
	uint16_t shooter_heat1_cooling_rate;//������ 42mm �ӵ�������ȴ�ٶ� ��λ /s 
	uint16_t shooter_heat1_cooling_limit;//������ 42mm �ӵ��������� 
	uint8_t  mains_power_gimbal_output:1;//gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v ���
	uint8_t  mains_power_chassis_output:1;//chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v ���
	uint8_t  mains_power_shooter_output:1;//shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v ���
}ext_game_robot_state_t; //����������״̬(0��0201)

typedef __packed struct
{
	uint16_t chassis_volt;//���������ѹ ��λ ���� 
	uint16_t chassis_current;//����������� ��λ ���� 
	float    chassis_power;//����������� ��λ W �� 
	uint16_t chassis_power_buffer;//���̹��ʻ��� ��λ J ���� 
	uint16_t chassis_shooter_heat0;//17mm ǹ������ 
	uint16_t chassis_shooter_heat1;//42mm ǹ������ 
}ext_power_heat_data_t; //ʵʱ������������(0��0202)

typedef __packed struct
{
	//��λm
	float x;
	float y;
	float z;
	float yaw;//λ��ǹ�ڣ���λ�� 
}ext_game_robot_pos_t; //������λ��(0��0203)

typedef __packed struct
{
	uint8_t power_rune_buff;
/*bit 0��������Ѫ����Ѫ״̬ 
	bit 1��ǹ��������ȴ���� 
	bit 2�������˷����ӳ� 
	bit 3�������˹����ӳ� 
	���� bit ���� */
}ext_buff_musk_t; //����������(0��0204)

typedef __packed struct
{
	uint8_t energy_point;//���۵������� 
	uint8_t attack_time;//�ɹ���ʱ�� ��λ s��50s �ݼ��� 0 
}aerial_robot_energy_t; //���л���������״̬(0��0205)

typedef __packed struct
{
	uint8_t armor_id:4;
//bit 0-3����Ѫ���仯����Ϊװ���˺�������װ�� ID��������ֵΪ 0-4 �Ŵ�������˵����װ��Ƭ������Ѫ���仯���ͣ��ñ�����ֵΪ 0�� 
	uint8_t hurt_type:4;
/*bit 4-7��Ѫ���仯���� 
	0x0 װ���˺���Ѫ�� 
	0x1 ģ����߿�Ѫ�� 
	0x2 ��ǹ��������Ѫ�� 
	0x3 �����̹��ʿ�Ѫ�� */
}ext_robot_hurt_t; //�˺�״̬(0��0206)

typedef __packed struct
{
	uint8_t bullet_type;//�ӵ�����: 1��17mm ���� 2��42mm ���� 
	uint8_t bullet_freq;//�ӵ���Ƶ ��λ Hz 
	float   bullet_speed;//�ӵ����� ��λ m/s 
}ext_shoot_data_t; //ʵʱ�����Ϣ(0��0207)

//�����˼佻������

typedef __packed struct
{
	float   data1;//�Զ��帡������ 1
	float   data2;//�Զ��帡������ 2
	float   data3;//�Զ��帡������ 3
	uint8_t masks;//bit 0-5���ֱ���ƿͻ����Զ���������ʾ����ϵ�����ָʾ �ƣ�ֵΪ 1ʱ��ʾ��ɫ��ֵΪ 0����ʾ��ɫ�� Bit 6-7������ 
}client_custom_data_t;//�ͻ��� (�ͻ����Զ������ݣ�cmd_id:0x0301������ID:0xD180)

typedef __packed struct
{
	uint8_t data[Interactive_Data_Len];
}robot_interactive_data_t;//ѧ�������˼�ͨ��(�ͻ����Զ������ݣ�cmd_id:0x0301������ID:0x0201~0x02FF)

typedef __packed struct
{
	__packed struct
	{
	uint16_t data_cmdid;//����ID
	uint16_t send_ID;//��ҪУ�鷢���ߵ� ID ��ȷ�ԣ������ 1 ���͸��� 5�������� ҪУ��� 1 
	uint16_t receiver_ID;//��ҪУ������ߵ� ID ��ȷ�ԣ����粻�ܷ��͵��жԻ����˵� ID 
	}ext_student_interative_header_data_t;//���ݶ�ͷ�ṹ
/*ID˵����1.����ID�������ģ������ͻ����Զ�������
	������������������������漺�������˼�ͨ��
	��.���ͻ�����ߵ�ID��
1. ������ ID��1��Ӣ��(��)��2������(��)��3/4/5������(��)��6������(��)��7���ڱ�(��)��11��Ӣ��(��)�� 12������(��)��13/14/15������(��)��16������(��)��17���ڱ�(��)�� 
2. �ͻ��� ID��0x0101 ΪӢ�۲����ֿͻ���(��)��0x0102�� ���̲����ֿͻ���((��)�� 0x0103/0x0104/0x0105�����������ֿͻ���(��)��0x0106�����в����ֿͻ���((��)�� 0x0111��Ӣ �۲����ֿͻ���(��)��0x0112�����̲����ֿͻ���(��)��0x0113/0x0114/0x0115�������ֿͻ��˲� ��(��)��0x0116�����в����ֿͻ���(��)
*/	
	__packed union
  {	
		client_custom_data_t       client_custom_data;//����ID:(0xD108) �ͻ���
		robot_interactive_data_t   robot_interactive_data;//����ID:(0x0201~0x02FF) ѧ�������˼�ͨ��
	}ReceiveData;//������������
}ext_student_interative_t;//�������ݽ�����Ϣ(0x0301)

/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
    ext_game_state_t                game_state;//(0x0001)����״̬��Ϣ
    ext_game_result_t               game_result;//(0x0002)�����������
    ext_game_robot_survivors_t      game_robot_survivors;//(0x0003)�����˴������
    ext_event_data_t                event_data;//(0��0101)�����¼�����
    ext_supply_projectile_action_t  supply_projectile_action; //(0��0102)����վ������ʶ
    ext_supply_projectile_booking_t supply_projectile_booking;//(0��0103)����վԤԼ�ӵ�
    ext_game_robot_state_t          game_robot_state;//(0��0201)����������״̬
    ext_power_heat_data_t           power_heat_data;//(0��0202)ʵʱ������������
    ext_game_robot_pos_t            game_robot_pos; //(0��0203)������λ��
    ext_buff_musk_t                 buff_musk; //(0��0204)����������
    aerial_robot_energy_t           aerial_robot_energy; //(0��0205)���л���������״̬
    ext_robot_hurt_t                robot_hurt; //(0��0206)�˺�״̬
    ext_shoot_data_t                shoot_data; //(0��0207)ʵʱ�����Ϣ
    ext_student_interative_t        student_interative;//(0x0301)�������ݽ�����Ϣ
} receive_judge_t;//����ϵͳ���ڽ��ܵ����ݽṹ��


void MucRefereeSystemInit( void );
void MucCallbackRefereeSystemHandle(unpack_data_t *p_obj, uint8_t* buff);
void data_upload_handler(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);

extern uint8_t g_judgeRxDataBuf[Judge_Buf_LEN];
extern unpack_data_t judge_unpack_obj;
extern receive_judge_t g_refereeSystemReceMesg;
extern uint8_t g_judgeTxDataBuf[100];
#endif

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "can_bsp.h"

typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
	CAN_GIMBAL_ALL_ID = 0x1FE,
	CAN_6020_M1_ID = 0x205,
	CAN_6020_M2_ID = 0x206,
	CAN_TRIGGER_ID = 0x207,
}Can_msg_id_e;				//C620������ID����

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} Motor_dat_t;			//�������

typedef struct
{
	int32_t round_cnt;		//���ת��Ȧ��
	int32_t total_angle;	//���һ��ת�ĽǶ�
	int16_t offset_angle;	//��ʼ�ϵ�Ƕ�
	
	float angle;			//�����ת�ĽǶ�
}Motor_Angle;				//����ļ���ת�Ƕ�����

void Cale_Trigger_angle_t(Motor_dat_t *motor,Motor_Angle *motor_data);
void Cale_Gimbal_angle_t(Motor_dat_t *motor,Motor_Angle *motor_data);
void Motor_dat_transmit(Motor_dat_t *motor_t, uint8_t *Motor_dat);
void Fdcan1_callback(FDCAN_HandleTypeDef *hfdcan);

void Chassis_cmd_CAN(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void Gimbal_cmd_CAN(int16_t motor1, int16_t motor2);
void Shoot_cmd_CAN(int16_t motor1, int16_t motor2, int16_t motor3);

void Fdcan2_callback(FDCAN_HandleTypeDef *hfdcan);

Motor_dat_t *get_yaw_gimbal_motor_measure_point(void);
Motor_dat_t *get_pitch_gimbal_motor_measure_point(void);
Motor_dat_t *get_trigger_motor_measure_point(void);
Motor_dat_t *get_chassis_motor_measure_point(uint8_t i);
Motor_dat_t *get_shooter_motor_measure_point(uint8_t i);

#endif

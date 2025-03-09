#ifndef __CHASSIS_TASK__
#define __CHASSIS_TASK__

#include "Gimbal_task.h"
#include "pid.h"
#include "remoter.h"
#include "user_lib.h"
#include "can_bsp.h"
#include "bsp_usart.h"
#include "Motor.h"


//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 500
//������ʱ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//������ʱ�� 2ms
#define CHASSIS_CONTROL_TIME 0.002f
//�����������Ƶ�� 
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//ǰ���ң����ͨ������ ��ch
#define CHASSIS_X_CHANNEL 3
//���ҵ�ң����ͨ������ ��ch
#define CHASSIS_Y_CHANNEL 2
//������ģʽ�£�����ͨ��ң����������ת ��ch
#define CHASSIS_WZ_CHANNEL 0

//ѡ�����״̬ ����ͨ�� s[0]������̨ͬʱ����
#define CHASSIS_MODE_CHANNEL 0

//ҡ������
#define CHASSIS_RC_DEADLINE 10

//����ҡ�ڰ���
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D


/*********************************************************************************/
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 15
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 15

//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 3

//�������̵������ٶ�
#define MAX_WHEEL_SPEED 2000.0f

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 800.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 800.0f
//������ת���ٶ�ת������
#define CHASSIS_WZ_SET_SCALE 0.3f

//m3508ת���ɵ����ٶ�(m/s)�ı������ڴ����ǲ�����ʵ���ٶ�ת����ֱ��ͨ�����ת������ǻ������ٶ�
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 1
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 1
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 1
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 1
#define MOTOR_DISTANCE_TO_CENTER		1

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f


/*********************************************************************************/


//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f
//	float PID_Chassis_Follow_Control_args[3] = {20, 0, 0.2};
//	float PID_Chassis_Follow_Gimbal_Set[3] = {800,0,0.1};
	
//���̵���ٶȻ�PID
#define M3508_MOTOR_SPEED_PID_KP 10.0f
#define M3508_MOTOR_SPEED_PID_KI 0.1f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT 16383.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 8000.0f
//���̵����̨����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.2f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 2000.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 3000.0f

#ifndef PI
#define PI 3.14159f 
#endif

typedef enum
{
	//����ģʽ
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,	//���̸�����̨
	CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,	//�����Լ��ǶȻ�
	CHASSIS_VECTOR_NO_FOLLOW_YAW,		//�����ɰ���ֱ�ӿ���
	CHASSIS_VECTOR_RAW,					//����ԭʼ״̬
}chassis_mode_e;

typedef struct
{
	const Motor_dat_t *chassis_motor_measure;
	float accel;
	float speed;
	float speed_set;
	int16_t given_current;					//Ť�ص���
}chassis_motor_t;

typedef struct
{
	const RC_ctrl_t *chassis_RC;
	const gimbal_motor_t *chassis_yaw_motor;
	const gimbal_motor_t *chassis_pitch_motor;
	chassis_mode_e chassis_mode;							//����ģʽ
	chassis_mode_e last_chassis_mode;
	chassis_motor_t motor_chassis[4];
	
	pid_type_def motor_speed_pid[4];
	pid_type_def chassis_angle_pid;
	
	first_order_filter_type_t chassis_cmd_slow_set_vx;	//һ�׵�ͨ�˲�����vx�趨ֵ
	first_order_filter_type_t chassis_cmd_slow_set_vy;	//һ�׵�ͨ�˲�����vy�趨ֵ
	
	float vx;
	float vy;
	float wz;
	float vx_set;
	float vy_set;
	float wz_set;
	float chassis_relative_angle;
	float chassis_relative_angle_set;
	float chassis_yaw_set;			// ���ڵ��̸���
	
	float vx_max_speed;
	float vx_min_speed;
	float vy_max_speed;
	float vy_min_speed;
	float wz_max_speed;
	float wz_min_speed;
	
	float chassis_yaw;				//�����Ǻ���̨������ӵ�yaw�Ƕ�
	float chassis_yaw_positive;	//YAW��̨�������Ƕ�λ��

}chassis_move_t;

void chassis_mode_change_control_transit(chassis_move_t * chassis_move_transit);
void chassis_set_mode(chassis_move_t *chassis_move_mode);
void chassis_vector_to_mecanum_wheel_speed(float vx_set, float vy_set, float wz_set, float wheel_speed[4]);
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
void chassis_feedback_update(chassis_move_t *chassis_move_update);
void chassis_init(chassis_move_t *chassis_move_init);
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_set_control(chassis_move_t *chassis_move_control);

#endif

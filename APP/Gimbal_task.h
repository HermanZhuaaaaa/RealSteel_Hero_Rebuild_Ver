#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "Motor.h"
#include "pid.h"
#include "BMI088driver.h"
#include "remoter.h"

//��̨�����Ƿ��Ƿ���
#define PITCH_TURN  1
#define YAW_TURN    0

//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 500
//��̨������ 1ms
#define GIMBAL_CONTROL_TIME_MS 1
//yaw,pitch����ͨ��
#define GIMBAL_YAW_CHANNEL 0
#define GIMBAL_PITCH_CHANNEL 1
//ѡ����̨״̬ ����ͨ�� s[0]�������ͬʱ����
#define GIMBAL_MODE_CHANNEL 0
//ҡ������
#define GIMBAL_RC_DEADLINE 10
/**************************************************/

#define YAW_RC_SEN    0.0008f
#define PITCH_RC_SEN  0.0008f //0.005

#define YAW_MOUSE_SEN   0.0008f
#define PITCH_MOUSE_SEN 0.0004f

#define YAW_ENCODE_SEN    1.01f
#define PITCH_ENCODE_SEN  1.11f
/**************************************************/
//	float PID_Gimbal_Yaw_args[2][3] = {{-20, 0, 0},{80, 0, 0}};
//	float PID_Gimbal_Pitch_args[2][3] = {{25, 0, 100},{80, 0, 0}};

//pitch �ٶȻ� PID����
#define PITCH_SPEED_PID_KP        40.0f
#define PITCH_SPEED_PID_KI        0.1f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   25000.0f
#define PITCH_SPEED_PID_MAX_IOUT  16383.0f
//yaw   �ٶȻ� PID����
#define YAW_SPEED_PID_KP        40.0f
#define YAW_SPEED_PID_KI        0.1f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   25000.0f
#define YAW_SPEED_PID_MAX_IOUT  10000.0f

//pitch �ǶȻ� �Ƕ��������ǽ��� PID����
#define PITCH_GYRO_ABSOLUTE_PID_KP 			15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 			0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 			100.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 	300.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 	0.0f
//yaw �ǶȻ� �Ƕ��������ǽ��� PID����
#define YAW_GYRO_ABSOLUTE_PID_KP        1600.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        1000.5f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   300.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

//pitch �ǶȻ� �Ƕ��ɱ����� PID����
#define PITCH_ENCODE_RELATIVE_PID_KP 		-10.5f
#define PITCH_ENCODE_RELATIVE_PID_KI 		0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 		0.8f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 	300.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 	0.0f
//yaw �ǶȻ� �Ƕ��ɱ����� PID����
#define YAW_ENCODE_RELATIVE_PID_KP        1.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   700.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

//�������ֵ����Լ���ֵ
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//�������ֵת���ɽǶ�ֵ
#define MOTOR_ECD_TO_RAD 0.000766990394f //  2*PI/8192

//��̨��ʼ������ֵ����������,��������Χ��ֹͣһ��ʱ���Լ����ʱ��6s������ʼ��״̬��
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//��̨��ʼ������ֵ���ٶ��Լ����Ƶ��ĽǶ�
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

//��ͷ180 ����
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//��ͷ��̨�ٶ�
#define TURN_SPEED    5.0f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f
//��̨�Ƕ�����
#define GIMBAL_CONTROL_ABSOLUTE_DEADZONE 1.0f
//�ֶ����Գ�����̨��ʼ������
#define PITCH_LOW_ECD 6936
#define PITCH_HIGH_ECD 6227
#define PITCH_MID_ECD 6800

#define PITCH_MIN 147
#define PITCH_MAX 182
#define PITCH_MID 180

#define YAW_INIT_ECD 4240

typedef enum
{
	GIMBAL_MOTOR_RAW = 0,	//���ԭʼֵ����
	GIMBAL_MOTOR_GYRO,		//����������ǽǶȿ���
	GIMBAL_MOTOR_ENCODER,	//����ɱ������Ƕȿ���
}gimbal_motor_mode_e;

typedef enum
{
	GIMBAL_MOTOR_YAW = 0,
	GIMBAL_MOTOR_PITCH,
}gimbal_motor_class_e;

typedef struct
{
	gimbal_motor_class_e motor_class;
	float kp;
	float ki;
	float kd;
	
	float set;
	float get;
	float err;
	
	float max_out;
	float max_iout;
	
	float Pout;
	float Iout;
	float Dout;
	
	float out;
}gimbal_PID_t;

typedef struct
{

	const Motor_dat_t *gimbal_motor_measure;		//���ԭʼ����
	gimbal_motor_mode_e gimbal_motor_mode;
	gimbal_motor_mode_e last_gimbal_motor_mode;
	
	gimbal_PID_t gimbal_motor_absolute_angle_pid;
	gimbal_PID_t gimbal_motor_relative_angle_pid;
	pid_type_def gimbal_motor_gyro_pid;

	uint16_t offset_ecd;
	float max_relative_angle;
	float min_relative_angle;
	
	float relative_angle;
	float relative_angle_set;
	float absolute_angle;
	float absolute_angle_set;
	
	float motor_gyro;
	float motor_gyro_set;
	
	float motor_speed;
	float raw_cmd_current;
	float current_set;
	
	int16_t given_current;	//Ť�ص���
}gimbal_motor_t;

typedef struct
{
	float max_yaw;
	float min_yaw;
	float max_pitch;
	float min_pitch;
	
	uint16_t max_yaw_ecd;
	uint16_t min_yaw_ecd;
	uint16_t max_pitch_ecd;
	uint16_t min_pitch_ecd;
}gimbal_step_cali_t;

typedef struct
{
	const RC_ctrl_t 	*gimbal_RC;
	
	float 				gimbal_yaw;			//��̨�����yaw�Ƕ�
	float 				gimbal_pitch;		//��̨�����pitch�Ƕ�
	float 				gimbal_roll;		//��̨�����roll�Ƕ�
	float				gimbal_gyro[3];		//��̨������ٶ�
	
	gimbal_motor_t 		gimbal_yaw_motor;
	gimbal_motor_t 		gimbal_pitch_motor;
	gimbal_step_cali_t 	gimbal_limitation;
}gimbal_control_t;

void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
void gimbal_control_loop(gimbal_control_t *gimbal_control_loop);
void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float add);
void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float add);
void gimbal_set_control(gimbal_control_t *gimbal_set_control);
void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_control_transit);
void gimbal_set_mode(gimbal_control_t *gimbal_set_mode);
const gimbal_motor_t *get_yaw_motor_point(void);
const gimbal_motor_t *get_pitch_motor_point(void);
void gimbal_init(gimbal_control_t *gimbal_control_init);
void gimbal_PID_init(gimbal_PID_t *pid, float kp, float ki, float kd, float max_out, float max_iout);
float gimbal_PID_calc(gimbal_PID_t *pid,float get,float set,float error_delta);
void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear);
void gimbal_feedback_update(gimbal_control_t *gimbal_feedback_update);
float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);


#endif

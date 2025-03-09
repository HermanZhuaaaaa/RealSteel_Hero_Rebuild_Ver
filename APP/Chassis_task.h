#ifndef __CHASSIS_TASK__
#define __CHASSIS_TASK__

#include "Gimbal_task.h"
#include "pid.h"
#include "remoter.h"
#include "user_lib.h"
#include "can_bsp.h"
#include "bsp_usart.h"
#include "Motor.h"


//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 500
//任务间隔时间 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//任务间隔时间 2ms
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率 
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//前后的遥控器通道号码 左ch
#define CHASSIS_X_CHANNEL 3
//左右的遥控器通道号码 左ch
#define CHASSIS_Y_CHANNEL 2
//在特殊模式下，可以通过遥控器控制旋转 右ch
#define CHASSIS_WZ_CHANNEL 0

//选择底盘状态 开关通道 s[0]，与云台同时更改
#define CHASSIS_MODE_CHANNEL 0

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D


/*********************************************************************************/
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 15
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 15

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 3

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 2000.0f

//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 800.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 800.0f
//底盘旋转角速度转换比例
#define CHASSIS_WZ_SET_SCALE 0.3f

//m3508转化成底盘速度(m/s)的比例，在此我们不进行实际速度转换，直接通过电机转速来标记机器人速度
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 1
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 1
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 1
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 1
#define MOTOR_DISTANCE_TO_CENTER		1

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f


/*********************************************************************************/


//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f
//	float PID_Chassis_Follow_Control_args[3] = {20, 0, 0.2};
//	float PID_Chassis_Follow_Gimbal_Set[3] = {800,0,0.1};
	
//底盘电机速度环PID
#define M3508_MOTOR_SPEED_PID_KP 10.0f
#define M3508_MOTOR_SPEED_PID_KI 0.1f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT 16383.0f
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 8000.0f
//底盘电机云台跟随PID
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
	//底盘模式
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,	//底盘跟随云台
	CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,	//底盘自己角度环
	CHASSIS_VECTOR_NO_FOLLOW_YAW,		//底盘由按键直接控制
	CHASSIS_VECTOR_RAW,					//底盘原始状态
}chassis_mode_e;

typedef struct
{
	const Motor_dat_t *chassis_motor_measure;
	float accel;
	float speed;
	float speed_set;
	int16_t given_current;					//扭矩电流
}chassis_motor_t;

typedef struct
{
	const RC_ctrl_t *chassis_RC;
	const gimbal_motor_t *chassis_yaw_motor;
	const gimbal_motor_t *chassis_pitch_motor;
	chassis_mode_e chassis_mode;							//运行模式
	chassis_mode_e last_chassis_mode;
	chassis_motor_t motor_chassis[4];
	
	pid_type_def motor_speed_pid[4];
	pid_type_def chassis_angle_pid;
	
	first_order_filter_type_t chassis_cmd_slow_set_vx;	//一阶低通滤波减缓vx设定值
	first_order_filter_type_t chassis_cmd_slow_set_vy;	//一阶低通滤波减缓vy设定值
	
	float vx;
	float vy;
	float wz;
	float vx_set;
	float vy_set;
	float wz_set;
	float chassis_relative_angle;
	float chassis_relative_angle_set;
	float chassis_yaw_set;			// 用于底盘跟随
	
	float vx_max_speed;
	float vx_min_speed;
	float vy_max_speed;
	float vy_min_speed;
	float wz_max_speed;
	float wz_min_speed;
	
	float chassis_yaw;				//陀螺仪和云台电机叠加的yaw角度
	float chassis_yaw_positive;	//YAW云台正方向电角度位置

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

#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#include "user_lib.h"
#include "stdbool.h"
#include "remoter.h"
#include "Motor.h"
#include "pid.h"

//任务初始化 空闲一段时间
#define SHOOT_TASK_INIT_TIME 500
//云台任务间隔 1ms
#define SHOOT_CONTROL_TIME_MS 1

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//射击开启按键（摩擦轮）
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//鼠标长按判断
#define PRESS_LONG_TIME 400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME 2000
//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//摩擦轮高速 加速 时间
#define UP_ADD_TIME 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18
//拨弹速度
#define TRIGGER_SPEED 10.0f
#define CONTINUE_TRIGGER_SPEED 15.0f
#define READY_TRIGGER_SPEED 5.0f
//摩擦轮转速
#define SHOOTER_SPEED_RPM_SET 6000


#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f


#define PI_TEN 0.314f
#define PI_THR 1.0471975511965977461542144610932f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        100.0f
#define TRIGGER_ANGLE_PID_KI        0.1f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  16383.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 10000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f

//摩擦轮电机速度环PID
#define SHOOTER_SPEED_PID_KP 10.0f
#define SHOOTER_SPEED_PID_KI 0.1f
#define SHOOTER_SPEED_PID_KD 0.0f
#define SHOOTER_SPEED_PID_MAX_OUT 16383.0f
#define SHOOTER_SPEED_PID_MAX_IOUT 8000.0f

typedef enum
{
	SHOOT_STOP = 0,
	SHOOT_READY_FRIC,
	SHOOT_READY_BULLET,
	SHOOT_READY,
	SHOOT_BULLET,
	SHOOT_CONTINUE_BULLET,
	SHOOT_DONE
}shoot_mode_e;


typedef struct
{
	shoot_mode_e shoot_mode;
	const RC_ctrl_t *shoot_RC;
	const Motor_dat_t *shoot_motor_measure[2];
	const Motor_dat_t *trigger_motor_measure;
	
	ramp_function_source_t *fric1_ramp;
	uint16_t fric_pwm1;
	
	ramp_function_source_t *fric2_ramp;
	uint16_t fric_pwm2;
	
	pid_type_def trigger_motor_pid;
	pid_type_def shooter_motor_pid[2];
	float trigger_speed_set;
	float speed;
	float speed_set;
	
	float angle;
	float angle_set;
	
	int16_t given_current;
	int8_t ecd_count;
	
	bool press_l;
	bool press_r;
	bool last_press_l;
	bool last_press_r;
	uint16_t press_l_time;
	uint16_t press_r_time;
	uint16_t rc_s_time;
	
	uint16_t block_time;
	uint16_t reverse_time;
	bool move_flag;
	
	bool key;
	uint8_t key_time;
	
	uint16_t heat_limit;
	uint16_t heat;
}shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
void shoot_init(void);
//射击循环
void shoot_control_loop(void);
//射击数据更新
void shoot_feedback_update(void);
//射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
void shoot_set_mode(void);
//射击控制，控制拨弹电机角度，完成一次发射
void shoot_bullet_control(void);
//英雄无法堵转掉头，故不使用此代码，无需处理
void trigger_motor_turn_back(void);

#endif

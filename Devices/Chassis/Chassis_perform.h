#ifndef __CHASSIS_PERFORM_H
#define __CHASSIS_PERFORM_H

#include "Chassis_task.h"

#define CHASSIS_OPEN_LOOP_RC_SCALE 10	//开环遥控器控制电机系数

typedef enum
{
	//底盘电机状态
	CHASSIS_ZERO_FORCE,			//底盘无力，未上电的状态
	CHASSIS_NO_MOVE,			//底盘不动，电机固定角度
	CHASSIS_FOLLOW_GIMBAL_YAW,	//底盘跟随云台
	CHASSIS_NO_FOLLOW_YAW,		//底盘无角度，只有速度环
	CHASSIS_OPEN_LOOP			//底盘开环 遥控器数值乘系数直接发送电机
}chassis_behaviour_e;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
void chassis_behaviour_control_set(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_no_move_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_open_loop_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_little_top_move_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);
#endif

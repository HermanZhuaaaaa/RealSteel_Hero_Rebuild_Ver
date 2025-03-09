#ifndef __CHASSIS_PERFORM_H
#define __CHASSIS_PERFORM_H

#include "Chassis_task.h"

#define CHASSIS_OPEN_LOOP_RC_SCALE 10	//����ң�������Ƶ��ϵ��

typedef enum
{
	//���̵��״̬
	CHASSIS_ZERO_FORCE,			//����������δ�ϵ��״̬
	CHASSIS_NO_MOVE,			//���̲���������̶��Ƕ�
	CHASSIS_FOLLOW_GIMBAL_YAW,	//���̸�����̨
	CHASSIS_NO_FOLLOW_YAW,		//�����޽Ƕȣ�ֻ���ٶȻ�
	CHASSIS_OPEN_LOOP			//���̿��� ң������ֵ��ϵ��ֱ�ӷ��͵��
}chassis_behaviour_e;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);
void chassis_behaviour_control_set(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_no_move_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_open_loop_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
void chassis_little_top_move_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);
#endif

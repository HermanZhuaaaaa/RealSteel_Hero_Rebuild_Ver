#include "Chassis_perform.h"
#include "imu_temp_ctrl.h"
#include "remoter.h"

chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;	//底盘电机状态
extern chassis_move_t chassis_move;

/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	if(chassis_move_mode == NULL)
	{
		return;
	}
	
	//遥控器模式设置，右侧拨杆来设置模式 
	/**********************************
	 *1 up		云台跟随
	 *3 mid		电机不动
	 *2 down	单底盘速度环
	 **********************************
	 */
	if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
	}
	else if(switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		chassis_behaviour_mode = CHASSIS_NO_MOVE;
	}
	else if(switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
	{
		chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
	}
	
	//通过拨杆控制底盘电机状态behaviour，通过状态设置底盘模式mode
	if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 
    }
	else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
    }
	
	else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
    }
}

/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */
void chassis_behaviour_control_set(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if(vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	
	if(chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
	{
		chassis_follow_gimbal_yaw_control(vx_set,vy_set, angle_set,chassis_move_rc_to_vector);
	}
	else if(chassis_behaviour_mode == CHASSIS_NO_MOVE)
	{
		chassis_no_move_control(vx_set,vy_set,angle_set,chassis_move_rc_to_vector);
	}
	else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
	{
		chassis_no_follow_yaw_control(vx_set,vy_set,angle_set,chassis_move_rc_to_vector);
	}
}

/**
  * @brief          底盘不移动的行为下，底盘模式是不跟随角度，
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
void chassis_no_move_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if(vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	*vx_set = 0.0f;
	*vy_set = 0.0f;
	*wz_set = 0.0f;
}
/**
  * @brief          底盘跟随云台的行为下，底盘模式是跟随云台角度，
  *					底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
void chassis_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if(vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	//通过遥控器数据得出一般状态下的运动速度
	chassis_rc_to_control_vector(vx_set,vy_set,chassis_move_rc_to_vector);
}
/**
  * @brief          底盘不跟随角度的行为下，底盘模式是不跟随角度，
  *					底盘旋转速度由参数直接设定
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if(vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	
	//通过遥控器数据得出一般状态下的运动速度
	chassis_rc_to_control_vector(vx_set,vy_set,chassis_move_rc_to_vector);
	*wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}
/**
  * @brief          底盘开环的行为下，底盘模式是raw原生状态，
  *					故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         none
  */
void chassis_open_loop_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if(vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	*vx_set =  chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL]  * CHASSIS_OPEN_LOOP_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL]  * CHASSIS_OPEN_LOOP_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_LOOP_RC_SCALE;
    return;
}

/**
  * @brief          底盘进行小陀螺旋转运动
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
void chassis_little_top_move_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if(vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	//代码未完成
}

#include "Gimbal_perform.h"
#include "arm_math.h"
#include "user_lib.h"

//云台状态
gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief          云台行为状态机设置.
  * @param[in]      gimbal_mode_set: 云台数据指针
  * @retval         none
  */
void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set)
{
	if(gimbal_mode_set == NULL)
	{
		return;
	}
	//拨杆开关控制云台状态
	//云台跟随
	if(switch_is_up(gimbal_mode_set->gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
	}
	//不动
	else if(switch_is_mid(gimbal_mode_set->gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		gimbal_behaviour = GIMBAL_ZERO_FORCE;
	}
	//自己的角度环
	else if(switch_is_down(gimbal_mode_set->gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
	}
}

/**
  * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  * @param[out]     gimbal_mode_set: 云台数据指针
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
	if(gimbal_mode_set == NULL)
	{
		return;
	}
	//云台行为状态设置
	gimbal_behaviour_set(gimbal_mode_set);
    //根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCODER;
    }
}
/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
  * @param[in]      gimbal_mode_set:云台数据指针
  * @retval         none
  */
void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, gimbal_control_t *gimbal_control_set)
{
	if(gimbal_control_set == NULL || add_yaw == NULL || add_pitch == NULL)
	{
		return;
	}
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
}
/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
void gimbal_zero_force_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;
    rc_deadband_limit(gimbal_control_set->gimbal_RC->rc.ch[GIMBAL_YAW_CHANNEL], yaw_channel, GIMBAL_RC_DEADLINE);
    rc_deadband_limit(gimbal_control_set->gimbal_RC->rc.ch[GIMBAL_PITCH_CHANNEL], pitch_channel, GIMBAL_RC_DEADLINE);
    *yaw = -(yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_RC->mouse.x * YAW_MOUSE_SEN);
    *pitch = -(pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_RC->mouse.y * PITCH_MOUSE_SEN);
}
/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;
    rc_deadband_limit(gimbal_control_set->gimbal_RC->rc.ch[GIMBAL_YAW_CHANNEL], yaw_channel, GIMBAL_RC_DEADLINE);
    rc_deadband_limit(gimbal_control_set->gimbal_RC->rc.ch[GIMBAL_PITCH_CHANNEL], pitch_channel, GIMBAL_RC_DEADLINE);
    *yaw  = -(yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_RC->mouse.x * YAW_MOUSE_SEN);
    *pitch = -(pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_RC->mouse.y * PITCH_MOUSE_SEN);
}

/**
  * @brief          云台在某些行为下，需要底盘不动
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
bool gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          云台在某些行为下，需要射击停止
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
bool gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

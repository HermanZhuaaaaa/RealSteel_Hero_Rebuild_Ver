#include "Gimbal_perform.h"
#include "arm_math.h"
#include "user_lib.h"

//��̨״̬
gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ��Ϊ0��
  * @param          �����ң����ֵ
  * @param          ��������������ң����ֵ
  * @param          ����ֵ
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
  * @brief          ��̨��Ϊ״̬������.
  * @param[in]      gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */
void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set)
{
	if(gimbal_mode_set == NULL)
	{
		return;
	}
	//���˿��ؿ�����̨״̬
	//��̨����
	if(switch_is_up(gimbal_mode_set->gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
	}
	//����
	else if(switch_is_mid(gimbal_mode_set->gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		gimbal_behaviour = GIMBAL_ZERO_FORCE;
	}
	//�Լ��ĽǶȻ�
	else if(switch_is_down(gimbal_mode_set->gimbal_RC->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
	}
}

/**
  * @brief          ��gimbal_set_mode����������gimbal_task.c,��̨��Ϊ״̬���Լ����״̬������
  * @param[out]     gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
	if(gimbal_mode_set == NULL)
	{
		return;
	}
	//��̨��Ϊ״̬����
	gimbal_behaviour_set(gimbal_mode_set);
    //������̨��Ϊ״̬�����õ��״̬��
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
  * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  * @param[out]     add_yaw:���õ�yaw�Ƕ�����ֵ����λ rad
  * @param[out]     add_pitch:���õ�pitch�Ƕ�����ֵ����λ rad
  * @param[in]      gimbal_mode_set:��̨����ָ��
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
  * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
  *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
  * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      gimbal_control_set: ��̨����ָ��
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
  * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
  * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set:��̨����ָ��
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
  * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set: ��̨����ָ��
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
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
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
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
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

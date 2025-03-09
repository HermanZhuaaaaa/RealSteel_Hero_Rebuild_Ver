#include "Chassis_perform.h"
#include "imu_temp_ctrl.h"
#include "remoter.h"

chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;	//���̵��״̬
extern chassis_move_t chassis_move;

/**
  * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
  * @param[in]      chassis_move_mode: ��������
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	if(chassis_move_mode == NULL)
	{
		return;
	}
	
	//ң����ģʽ���ã��Ҳದ��������ģʽ 
	/**********************************
	 *1 up		��̨����
	 *3 mid		�������
	 *2 down	�������ٶȻ�
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
	
	//ͨ�����˿��Ƶ��̵��״̬behaviour��ͨ��״̬���õ���ģʽmode
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
  * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
  * @param[out]     wz_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
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
  * @brief          ���̲��ƶ�����Ϊ�£�����ģʽ�ǲ�����Ƕȣ�
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
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
  * @brief          ���̸�����̨����Ϊ�£�����ģʽ�Ǹ�����̨�Ƕȣ�
  *					������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
void chassis_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if(vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	//ͨ��ң�������ݵó�һ��״̬�µ��˶��ٶ�
	chassis_rc_to_control_vector(vx_set,vy_set,chassis_move_rc_to_vector);
}
/**
  * @brief          ���̲�����Ƕȵ���Ϊ�£�����ģʽ�ǲ�����Ƕȣ�
  *					������ת�ٶ��ɲ���ֱ���趨
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if(vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	
	//ͨ��ң�������ݵó�һ��״̬�µ��˶��ٶ�
	chassis_rc_to_control_vector(vx_set,vy_set,chassis_move_rc_to_vector);
	*wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}
/**
  * @brief          ���̿�������Ϊ�£�����ģʽ��rawԭ��״̬��
  *					�ʶ��趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶȣ���ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set ��ת�ٶȣ� ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
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
  * @brief          ���̽���С������ת�˶�
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
void chassis_little_top_move_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if(vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
		return;
	}
	//����δ���
}

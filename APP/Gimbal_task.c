#include "Gimbal_task.h"
#include "Gimbal_perform.h"
#include "arm_math.h"
#include "user_lib.h"
#include "imu_temp_ctrl.h"

#define TEST 0

//�������ֵ���� 0��8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }
//��̨��������
gimbal_control_t gimbal_control;
//���͵������ֵ
int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;
//IMU����
extern IMU_Data_t IMU_Data;

void gimbal_task_main(void *argument)
{
    /* USER CODE BEGIN gimbal_task_main */

    //��̨���ݳ�ʼ��
    gimbal_init(&gimbal_control);
    //��ʼ����ͣһС��
    //vTaskDelay(GIMBAL_TASK_INIT_TIME);

    /* Infinite loop */
    for (;;)
    {

        //������̨����ģʽ
        gimbal_set_mode(&gimbal_control);
        //ģʽ�л����ݱ���
        gimbal_mode_change_control_transit(&gimbal_control);
        //��̨���ݸ���
        gimbal_feedback_update(&gimbal_control);
        //������̨�������������ƶ���
        gimbal_set_control(&gimbal_control);
        //��̨ѭ��PID�������
        gimbal_control_loop(&gimbal_control);

#if TEST
        struct __packed Frame
        {
            fp32 fdata[2];
            uint8_t tail_data[4];
        };

        struct Frame frame = {.tail_data = {0x00, 0x00, 0x80, 0x7f}};
        frame.fdata[0] = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
        frame.fdata[1] = gimbal_control.gimbal_pitch_motor.absolute_angle;
        //HAL_UART_Transmit_DMA(&huart10,&frame,sizeof(frame));
#endif

#if YAW_TURN
        yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
        pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

        Gimbal_cmd_CAN(yaw_can_set_current, pitch_can_set_current);
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);
    }

    /* USER CODE END gimbal_task_main */
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //�ǶȻ� �ٶȻ� ����PID����
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, 0);
    gimbal_motor->current_set    = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->motor_gyro_set);
    //��ֵ
    gimbal_motor->given_current  = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //�ǶȻ� �ٶȻ� ����PID����

    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->gimbal_motor_measure->ecd, gimbal_motor->relative_angle_set, 0);
    gimbal_motor->current_set    = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->gimbal_motor_measure->speed_rpm, gimbal_motor->motor_gyro_set);
    //��ֵ
    gimbal_motor->given_current  = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_RAW������ֱֵ�ӷ��͵�CAN����.
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     gimbal_control_loop:"gimbal_control"����ָ��.
  * @retval         none
  */
void gimbal_control_loop(gimbal_control_t *gimbal_control_loop)
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }

    if (gimbal_control_loop->gimbal_yaw_motor. absolute_angle_set>360)
    {
        gimbal_control_loop->gimbal_yaw_motor. absolute_angle_set-=360;
    }
    else if (gimbal_control_loop->gimbal_yaw_motor. absolute_angle_set<0)
    {
        gimbal_control_loop->gimbal_yaw_motor. absolute_angle_set+=360;
    }

    //YAW���
    if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //RAWģʽ
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //GYROģʽ
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        //ENCODERģʽ
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }

    //PITCH���
    if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //RAWģʽ
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        if (gimbal_control_loop->gimbal_pitch_motor.absolute_angle_set < gimbal_control_loop->gimbal_limitation.max_pitch && gimbal_control_loop->gimbal_pitch_motor.absolute_angle_set > gimbal_control_loop->gimbal_limitation.min_pitch)
        {
            gimbal_control_loop->gimbal_pitch_motor.absolute_angle_set = gimbal_control_loop->gimbal_pitch_motor.absolute_angle_set;
        }
        else if (gimbal_control_loop->gimbal_pitch_motor.absolute_angle_set > gimbal_control_loop->gimbal_limitation.max_pitch)
        {
            gimbal_control_loop->gimbal_pitch_motor.absolute_angle_set = gimbal_control_loop->gimbal_limitation.max_pitch;
        }
        else if (gimbal_control_loop->gimbal_pitch_motor.absolute_angle_set < gimbal_control_loop->gimbal_limitation.min_pitch)
        {
            gimbal_control_loop->gimbal_pitch_motor.absolute_angle_set = gimbal_control_loop->gimbal_limitation.min_pitch;
        }

        //GYROģʽ
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        //ENCODERģʽ
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
}

/**
  * @brief          ������̨�����趨ֵ������ֵ��ͨ��gimbal_behaviour_control_set�������õ�
  * @param[out]     gimbal_set_control:"gimbal_control"����ָ��.
  * @retval         none
  */
void gimbal_set_control(gimbal_control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }

    float add_yaw_angle   = 0.0f;
    float add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);

    //yaw���ģʽ����
    if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        //encoderģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    //pitch���ģʽ����
    if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyroģʽ�£������ǽǶȿ���
        gimbal_absolute_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER)
    {
        //encoderģʽ�£��������Ƕȿ���
        gimbal_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_GYRO��ʹ�������Ǽ����ŷ���ǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    float offset_angle  = 0.0f;
    float angle_set     = 0.0f;
    //��ǰ���Ƕ�
    offset_angle = gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle;
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = angle_set + add;
}

/**
  * @brief          ��̨����ģʽ:GIMBAL_MOTOR_ENCONDE��ʹ�ñ�����Խǽ��п���
  * @param[out]     gimbal_motor:yaw�������pitch���
  * @retval         none
  */
void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    float offset_angle  = 0.0f;
    float angle_set     = 0.0f;
    //��ǰ���Ƕ�
    offset_angle = gimbal_motor->relative_angle_set - gimbal_motor->relative_angle;
    angle_set = gimbal_motor->relative_angle_set;
    gimbal_motor->relative_angle_set = angle_set + add;

    //�Ƿ񳬹������Сֵ
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }

}

/**
  * @brief          ��̨ģʽ�ı䣬��Щ������Ҫ�ı䣬�������yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰyaw�Ƕ�
  * @param[out]     gimbal_control_transit:"gimbal_control"����ָ��.
  * @retval         none
  */
void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_control_transit)
{
    if (gimbal_control_transit == NULL)
    {
        return;
    }

    //ģʽδ�л� ��ֱ�ӷ���
    if (gimbal_control_transit->gimbal_yaw_motor.last_gimbal_motor_mode == gimbal_control_transit->gimbal_yaw_motor.gimbal_motor_mode && gimbal_control_transit->gimbal_pitch_motor.last_gimbal_motor_mode == gimbal_control_transit->gimbal_pitch_motor.gimbal_motor_mode)
    {
        return;
    }

    //YAW�������RAWģʽ
    if ((gimbal_control_transit->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW) &&(gimbal_control_transit->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW))
    {
        gimbal_control_transit->gimbal_yaw_motor.raw_cmd_current = gimbal_control_transit->gimbal_yaw_motor.current_set = gimbal_control_transit->gimbal_yaw_motor.given_current;
    }
    //YAW�������GYROģʽ�������ǣ�
    else if ((gimbal_control_transit->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO) &&(gimbal_control_transit->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO))
    {
        gimbal_control_transit->gimbal_yaw_motor.absolute_angle_set = gimbal_control_transit->gimbal_yaw_motor.absolute_angle + 180;
    }
    //YAW�������ENCODERģʽ����������Ƕȣ�
    else if ((gimbal_control_transit->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCODER) &&(gimbal_control_transit->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER))
    {
        gimbal_control_transit->gimbal_yaw_motor.relative_angle_set = gimbal_control_transit->gimbal_yaw_motor.relative_angle;
    }

    //����ģʽ
    gimbal_control_transit->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_control_transit->gimbal_yaw_motor.gimbal_motor_mode;

    //PITCH�������RAWģʽ
    if ((gimbal_control_transit->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW) &&(gimbal_control_transit->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW))
    {
        gimbal_control_transit->gimbal_pitch_motor.raw_cmd_current = gimbal_control_transit->gimbal_pitch_motor.current_set = gimbal_control_transit->gimbal_pitch_motor.given_current;
    }
    //PITCH�������GYROģʽ�������ǣ�
    else if ((gimbal_control_transit->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO) &&(gimbal_control_transit->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO))
    {
        gimbal_control_transit->gimbal_pitch_motor.absolute_angle_set = gimbal_control_transit->gimbal_pitch_motor.absolute_angle;
    }
    //PITCH�������ENCODERģʽ����������Ƕȣ�
    else if ((gimbal_control_transit->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCODER) &&(gimbal_control_transit->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCODER))
    {
        gimbal_control_transit->gimbal_pitch_motor.relative_angle_set = gimbal_control_transit->gimbal_pitch_motor.relative_angle;
    }

    //����ģʽ
    gimbal_control_transit->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_control_transit->gimbal_pitch_motor.gimbal_motor_mode;
}

/**
  * @brief          ������̨����ģʽ����Ҫ��'gimbal_behaviour_mode_set'�����иı�
  * @param[out]     gimbal_set_mode:"gimbal_control"����ָ��.
  * @retval         none
  */
void gimbal_set_mode(gimbal_control_t *gimbal_set_mode)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }

    gimbal_behaviour_mode_set(gimbal_set_mode);
}

/**
  * @brief          ����yaw �������ָ��
  * @param[in]      none
  * @retval         yaw���ָ��
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          ����pitch �������ָ��
  * @param[in]      none
  * @retval         pitch
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          ��ʼ��"gimbal_control"����������pid��ʼ���� ң����ָ���ʼ������̨���ָ���ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     init:"gimbal_control"����ָ��.
  * @retval         none
  */
void gimbal_init(gimbal_control_t *gimbal_control_init)
{
    const float pitch_speed_pid[3]          = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    const float yaw_speed_pid[3]            = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    const float pitch_absolute_angle_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD};
    const float yaw_absolute_angle_pid[3]   = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD};
    const float pitch_relative_angle_pid[3] = {PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD};
    const float yaw_relative_angle_pid[3]   = {YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD};

    //��ȡ�������
    gimbal_control_init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    gimbal_control_init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //��ȡ�ϵ��Ƕ�
    gimbal_control_init->gimbal_yaw_motor.offset_ecd = gimbal_control_init->gimbal_yaw_motor.gimbal_motor_measure->ecd;
    gimbal_control_init->gimbal_pitch_motor.offset_ecd = gimbal_control_init->gimbal_pitch_motor.gimbal_motor_measure->ecd;

    //��ȡ��̨����������
    gimbal_control_init->gimbal_yaw     = IMU_Data.yaw;
    gimbal_control_init->gimbal_pitch   = IMU_Data.pitch;
    gimbal_control_init->gimbal_roll    = IMU_Data.roll;
    //��ȡң��������
    gimbal_control_init->gimbal_RC      = get_remote_control_point();
    //��ʼ�����ģʽ
    gimbal_control_init->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_control_init->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_control_init->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_control_init->gimbal_yaw_motor.gimbal_motor_mode;
    gimbal_control_init->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_control_init->gimbal_pitch_motor.gimbal_motor_mode;
    //��ʼ��Yaw���PID����
    gimbal_PID_init(&gimbal_control_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, yaw_absolute_angle_pid[0], yaw_absolute_angle_pid[1], yaw_absolute_angle_pid[2], YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_PID_init(&gimbal_control_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, yaw_relative_angle_pid[0], yaw_relative_angle_pid[1], yaw_relative_angle_pid[2], YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT);
    PID_init(&gimbal_control_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    gimbal_control_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid.motor_class = GIMBAL_MOTOR_YAW;
    gimbal_control_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.motor_class = GIMBAL_MOTOR_YAW;
    //��ʼ��Pitch���PID����
    gimbal_PID_init(&gimbal_control_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, pitch_absolute_angle_pid[0], pitch_absolute_angle_pid[1], pitch_absolute_angle_pid[2], PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
    gimbal_PID_init(&gimbal_control_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, -pitch_relative_angle_pid[0], pitch_relative_angle_pid[1], pitch_relative_angle_pid[2], PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);
    PID_init(&gimbal_control_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    gimbal_control_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid.motor_class = GIMBAL_MOTOR_PITCH;
    gimbal_control_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid.motor_class = GIMBAL_MOTOR_PITCH;
    //���pid������������Ϣ
    gimbal_total_pid_clear(gimbal_control_init);
    //��������
    gimbal_feedback_update(gimbal_control_init);

    gimbal_control_init->gimbal_yaw_motor.absolute_angle_set = gimbal_control_init->gimbal_yaw_motor.absolute_angle;
    gimbal_control_init->gimbal_yaw_motor.relative_angle_set = gimbal_control_init->gimbal_yaw_motor.relative_angle;
    gimbal_control_init->gimbal_yaw_motor.motor_gyro_set     = gimbal_control_init->gimbal_yaw_motor.motor_gyro;

    gimbal_control_init->gimbal_pitch_motor.absolute_angle_set = gimbal_control_init->gimbal_pitch_motor.absolute_angle;
    gimbal_control_init->gimbal_pitch_motor.relative_angle_set = gimbal_control_init->gimbal_pitch_motor.relative_angle;
    gimbal_control_init->gimbal_pitch_motor.motor_gyro_set     = gimbal_control_init->gimbal_pitch_motor.motor_gyro;

    gimbal_control_init->gimbal_limitation.max_pitch = PITCH_MAX;
    gimbal_control_init->gimbal_limitation.max_pitch_ecd = PITCH_HIGH_ECD;
    gimbal_control_init->gimbal_limitation.max_yaw = 360;
    gimbal_control_init->gimbal_limitation.max_yaw_ecd = 8191;
    gimbal_control_init->gimbal_limitation.min_pitch = PITCH_MIN;
    gimbal_control_init->gimbal_limitation.min_pitch_ecd = PITCH_HIGH_ECD;
    gimbal_control_init->gimbal_limitation.min_yaw = 0;
    gimbal_control_init->gimbal_limitation.min_yaw_ecd = 0;
    //YAW��PITCH������ֵ
    gimbal_control_init->gimbal_pitch_motor.max_relative_angle = PITCH_LOW_ECD;
    gimbal_control_init->gimbal_pitch_motor.min_relative_angle = PITCH_HIGH_ECD;

    gimbal_control_init->gimbal_yaw_motor.max_relative_angle = 8191;
    gimbal_control_init->gimbal_yaw_motor.min_relative_angle = 0;

    for (uint16_t i = 0; i < 2500; i++)
    {

        //PITCH���
        gimbal_PID_calc(&gimbal_control_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, (float)(gimbal_control_init->gimbal_pitch_motor.gimbal_motor_measure->ecd), PITCH_MID_ECD, 0);
        PID_calc(&gimbal_control_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, gimbal_control_init->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm, gimbal_control_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid.out);

        //YAW�� Ӧ��ת����  δ����
        gimbal_PID_calc(&gimbal_control_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, (float)(gimbal_control_init->gimbal_yaw_motor.gimbal_motor_measure->ecd), YAW_INIT_ECD, 0);
        PID_calc(&gimbal_control_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, gimbal_control_init->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm, gimbal_control_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid.out);
        Gimbal_cmd_CAN(0, gimbal_control_init->gimbal_pitch_motor.gimbal_motor_gyro_pid.out);
        vTaskDelay(2);
    }

    //���ǳ�ʼ�����PID����
    gimbal_PID_init(&gimbal_control_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, pitch_relative_angle_pid[0], pitch_relative_angle_pid[1], pitch_relative_angle_pid[2], PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);
}

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     gimbal_feedback_update:"gimbal_control"����ָ��.
  * @retval         none
  */
void gimbal_feedback_update(gimbal_control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }

    gimbal_feedback_update->gimbal_yaw = IMU_Data.yaw;
    gimbal_feedback_update->gimbal_pitch = IMU_Data.pitch;
    gimbal_feedback_update->gimbal_roll = IMU_Data.roll;
    //��̨���ݸ���
    gimbal_feedback_update->gimbal_pitch_motor.absolute_angle   = gimbal_feedback_update->gimbal_pitch;
#if PITCH_TURN
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle   = gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;
#else
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle   = gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd;
#endif
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro       = gimbal_feedback_update->gimbal_gyro[1];

    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle     = gimbal_feedback_update->gimbal_yaw;
#if YAW_TURN
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle     = gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
#else
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle     = gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd;
#endif
    //YAW����ٶ� PITCH����ٶ�
    gimbal_feedback_update->gimbal_gyro[0] = gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
    gimbal_feedback_update->gimbal_gyro[1] = gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
}

/**
  * @brief          ����ecd��offset_ecd֮�����ԽǶ�
  * @param[in]      ecd: �����ǰ����
  * @param[in]      offset_ecd: �����ֵ����
  * @retval         ��ԽǶȣ���λrad
  */
float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;

    if (relative_ecd > HALF_ECD_RANGE)
    {
        relative_ecd -= ECD_RANGE;
    }
    else if (relative_ecd < -HALF_ECD_RANGE)
    {
        relative_ecd += ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}


/**
  * @brief          ��ʼ��"gimbal_PID"����
  * @param[IN]      pid�ṹ��
  * @param[IN]      p,i,d����
  * @param[IN]      maxout��ֵ
  * @retval         none
  */
void gimbal_PID_init(gimbal_PID_t *pid, float kp, float ki, float kd, float max_out, float max_iout)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;
    pid->max_iout = max_iout;
    pid->max_out  = max_out;
}

float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set, float error_delta)
{
    float err;

    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->get = get;
    pid->set = set;
    err = set - get;
    pid->err = err;

    if (pid->motor_class == GIMBAL_MOTOR_YAW)
    {
        if (pid->err > 180) pid->err -= 360;

        if (pid->err < -180) pid->err += 360;

        if (pid->err < 190 && pid->err > 170 || pid->err > -190 && pid->err < -170) pid->err = 0;
    }

    pid->Pout = pid->kp * pid->err;
    pid->Iout = pid->ki * pid->err;
    pid->Dout = pid->kd * pid->err;
    abs_limit(pid->Iout, pid->max_iout);
    pid->out  = pid->Pout + pid->Iout + pid->Dout;

    if (pid->out > pid->max_out)
    {
        pid->out = pid->max_out;
    }
    else if (pid->out < -pid-> max_out)
    {
        pid->out = -pid->max_out;
    }

    //abs_limit(pid->out,pid->max_out);
    return pid->out;
}

/**
  * @brief          ��̨PID��������pid��out,iout
  * @param[out]     gimbal_pid_clear:"gimbal_control"����ָ��.
  * @retval         none
  */
void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }

    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

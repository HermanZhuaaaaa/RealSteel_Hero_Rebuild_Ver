#include "Shoot_task.h"
#include "Gimbal_perform.h"
#include "Motor.h"
shoot_control_t shoot_control;


void shoot_task_main(void *argument)
{
    /* USER CODE BEGIN shoot_task_main */

    vTaskDelay(SHOOT_TASK_INIT_TIME);
    shoot_init();

    /* Infinite loop */
    for (;;)
    {
        shoot_control_loop();
        vTaskDelay(SHOOT_CONTROL_TIME_MS);
    }

    /* USER CODE END shoot_task_main */
}

/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{
    const float trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    const float shooter_speed_pid[3] = {SHOOTER_SPEED_PID_KP, SHOOTER_SPEED_PID_KI, SHOOTER_SPEED_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //ң����ָ��
    shoot_control.shoot_RC = get_remote_control_point();
    //�������
    shoot_control.shoot_motor_measure[0] = get_shooter_motor_measure_point(0);
    shoot_control.shoot_motor_measure[1] = get_shooter_motor_measure_point(1);
    shoot_control.trigger_motor_measure = get_trigger_motor_measure_point();
    //PID��ʼ��
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.shooter_motor_pid[0], PID_POSITION, shooter_speed_pid, SHOOTER_SPEED_PID_MAX_OUT, SHOOTER_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control.shooter_motor_pid[1], PID_POSITION, shooter_speed_pid, SHOOTER_SPEED_PID_MAX_OUT, SHOOTER_SPEED_PID_MAX_IOUT);
    //��������
    shoot_feedback_update();
    //�ٲ���Ħ���ֵ����ͨ��PWM���Ƶ�
    //  ��ʹ������ٶȱ任��ֱ��ȫ�����
    //  ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    //  ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_DOWN, FRIC_OFF);
    //  shoot_control.fric_pwm1 = FRIC_OFF;
    //  shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.trigger_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.angle_set = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
}


/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */
void shoot_control_loop(void)
{
    shoot_set_mode();
    shoot_feedback_update();

    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
    }

    else if (shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        if (shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            //���ò����ֵĲ����ٶ�,��������ת��ת����Ӣ�۲��ܷ�ת��
            shoot_control.trigger_speed_set = READY_TRIGGER_SPEED;
            trigger_motor_turn_back();
        }
        else
        {
            shoot_control.trigger_speed_set = 0.0f;
            shoot_control.speed_set = 0.0f;
        }

        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //���ò����ֵ��ٶ�
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //���ò����ֵĲ����ٶ�,��������ת��ת�����޷�ת��
        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else if (shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    //����ɶ��˼
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.given_current = 0;
        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        //ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        //ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        PID_calc(&shoot_control.shooter_motor_pid[0], shoot_control.shoot_motor_measure[0]->speed_rpm, SHOOTER_SPEED_RPM_SET);
        PID_calc(&shoot_control.shooter_motor_pid[1], shoot_control.shoot_motor_measure[1]->speed_rpm, -SHOOTER_SPEED_RPM_SET);
    }
    else
    {
        //���㲦���ֵ��PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);

        if (shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }

        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
        //ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        //ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        PID_calc(&shoot_control.shooter_motor_pid[0], shoot_control.shoot_motor_measure[0]->speed_rpm, SHOOTER_SPEED_RPM_SET);
        PID_calc(&shoot_control.shooter_motor_pid[1], shoot_control.shoot_motor_measure[1]->speed_rpm, -SHOOTER_SPEED_RPM_SET);
    }

    //    shoot_control.fric_pwm1 = (uint16_t)(shoot_control.fric1_ramp.out);
    //    shoot_control.fric_pwm2 = (uint16_t)(shoot_control.fric2_ramp.out);
    //    shoot_fric1_on(shoot_control.fric_pwm1);
    //    shoot_fric2_on(shoot_control.fric_pwm2);
    //Shoot_cmd_CAN(shoot_control.shooter_motor_pid[0].out,shoot_control.shooter_motor_pid[1].out,shoot_control.given_current);
    //return shoot_control.given_current;
    Shoot_cmd_CAN(0, 0, shoot_control.given_current);

}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
void shoot_set_mode(void)
{
    int8_t last_s = RC_SW_UP;

    //�ϲ��жϣ�һ�ο������ٴιر�
    if ((switch_is_up(shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_up(shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //�����е���ʹ�ü��̿��Ʋ����� ������
    if (switch_is_mid(shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_RC->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    //�����е��� ����ʹ�ü��̹ر�Ħ����
    else if (switch_is_mid(shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_RC->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //ʲô��˼��
    if (shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp->out == shoot_control.fric1_ramp->max_value && shoot_control.fric2_ramp->out == shoot_control.fric2_ramp->max_value)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_BULLET && shoot_control.key == SWITCH_TRIGGER_ON)
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY && shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_READY_BULLET;
    }
    //ʲô��˼��

    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //�²�һ�λ�����갴��һ�Σ��������״̬
        if ((switch_is_down(shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }
    else if (shoot_control.shoot_mode == SHOOT_DONE)
    {
        if (shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            shoot_control.key_time++;

            if (shoot_control.key_time > SHOOT_DONE_KEY_OFF_TIME)
            {
                shoot_control.key_time = 0;
                shoot_control.shoot_mode = SHOOT_READY_BULLET;
            }
        }
        else
        {
            shoot_control.key_time = 0;
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }



    if (shoot_control.shoot_mode > SHOOT_READY_FRIC)
    {
        //��곤��һֱ�������״̬ ��������
        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }

    //���Բ���ϵͳ����
    //get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);

    //�����̨״̬�� ����״̬���͹ر����
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL];
}

/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
void shoot_feedback_update(void)
{
    float speed_filter_1 = 0.0f;
    float speed_filter_2 = 0.0f;
    float speed_filter_3 = 0.0f;
    //�����̵���ٶ��˲�����������
    const float filter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    //���׵�ͨ�˲�
    speed_filter_1 = speed_filter_2;
    speed_filter_2 = speed_filter_3;
    speed_filter_3 = speed_filter_2 * filter_num[0] + speed_filter_1 * filter_num[1] + (shoot_control.trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * filter_num[2];
    shoot_control.speed = speed_filter_3;

    //���Ȧ�����ã���Ϊ�������תһȦ�������ת36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (shoot_control.trigger_motor_measure->ecd - shoot_control.trigger_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count --;
    }
    else if (shoot_control.trigger_motor_measure->ecd - shoot_control.trigger_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count ++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    shoot_control.angle = (shoot_control.ecd_count*ECD_RANGE + shoot_control.trigger_motor_measure->ecd)*MOTOR_ECD_TO_ANGLE;
    //΢������
    //ɶjb����
    //shoot_control.key = BUTTEN_TRIG_PIN;
    //��갴��
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;

    //������ʱ
    if (shoot_control.press_l)//����������
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)//��һֱ��ʼ�������Ƶ�400
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)//�Ҽ�
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //��������µ�ʱ���ʱ
    //������ʹ��ң��������������Ա��� ͨ����ದ���ж� s[1]
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    //�ٲ����������������Ҽ����������Ӣ��ȡ�����趨�����ٸ������䲻��
    //��ʹ�����´��룬�Ȼῴ����ô��
    //  uint16_t up_time = 0;
    //    if (shoot_control.press_r)
    //    {
    //        up_time = UP_ADD_TIME;
    //    }

    //    if (up_time > 0)
    //    {
    //        shoot_control.fric1_ramp->max_value = FRIC_UP;
    //        shoot_control.fric2_ramp->max_value = FRIC_UP;
    //        up_time--;
    //    }
    //    else
    //    {
    //        shoot_control.fric1_ramp.max_value = FRIC_DOWN;
    //        shoot_control.fric2_ramp.max_value = FRIC_DOWN;
    //    }
}

/**
  * @brief          ��ת��ת����
  *                 Ӣ���޷���ת��ͷ���ʲ�ʹ�ô˴��룬���账��
  * @param[in]      void
  * @retval         void
  */
void trigger_motor_turn_back(void)
{
//    if (shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.speed_set = shoot_control.trigger_speed_set;
//    }
//    else
//    {
//        shoot_control.speed_set = -shoot_control.trigger_speed_set;
//    }

//    if (fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
//    {
//        shoot_control.block_time++;
//        shoot_control.reverse_time = 0;
//    }
//    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
//    {
//        shoot_control.reverse_time++;
//    }
//    else
//    {
//        shoot_control.block_time = 0;
//    }
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
void shoot_bullet_control(void)
{
    //ÿ�β���60�ȣ�1/3PI�ĽǶ�
    if (shoot_control.move_flag == 0)
    {
        shoot_control.angle_set = rad_format(shoot_control.angle + PI_TEN);
        shoot_control.move_flag = 1;
    }

    if (shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }

    //����Ƕ��ж�
    if (rad_format(shoot_control.angle_set - shoot_control.angle) > 0.05f)
    {
        //û����һֱ������ת�ٶ�
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
        //trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}


//void Shoot_Init(Shoot_Data_t *Shoot_Data_p)
//{
//    float PID_Shooter_args[2][3] = {{100, 0.1, 0}, {100, 0.1, 0}};
//    float PID_Trigger_args[2][3] = {{100, 0.1, 0}, {100, 0.1, 0}};
//    for (int i = 0; i < 2; i++)
//        PID_init(&(PID_Shooter[i]), PID_POSITION, PID_Shooter_args[i], 16383, 10000);
//    for (int i = 0; i < 2; i++)
//        PID_init(&(PID_Trigger[i]), PID_POSITION, PID_Trigger_args[i], 16383, 10000);
//
//    Shoot_Data_p->Shoot_RPM_Set[0] = 5000;
//    Shoot_Data_p->Shoot_RPM_Set[1] = -5000;
//      //TODO �������
//    //Shoot_Data_p->Trigger_Angle_Set = motor_trigger.angle;
//}

//void Shoot_Motor_Loop(Shoot_Data_t *Shoot_Data_p)
//{
//    PID_calc(&(PID_Shooter[0]), motor_shoot[0].speed_rpm, Shoot_Data.Shoot_RPM_Set[0]);
//  PID_calc(&(PID_Shooter[1]), motor_shoot[1].speed_rpm, Shoot_Data.Shoot_RPM_Set[1]);

//  switch (Shoot_Data_p->Shoot_Mode)
//  {
//      case SHOOT_SILENCE:
//            PID_calc(&(PID_Trigger[1]), motor_shoot[2].speed_rpm, 0);
//                      break;
////        δ��ɡ��ò���
////        case SHOOT_AIM_AUTO :
////                        break;
//      case SHOOT_ANGLE:
//            PID_calc(&(PID_Trigger[0]), motor_trigger.angle, Shoot_Data.Trigger_Angle_Set);
//                      PID_calc(&(PID_Trigger[1]), motor_shoot[2].speed_rpm, PID_Trigger[0].out);
//                      break;
//      case SHOOT_KEEP:
//            PID_calc(&(PID_Trigger[1]), motor_shoot[2].speed_rpm, -600);
//                      break;
////        Ӣ�����޷��˵���
////        case SHOOT_ROLLBACK :
////            PID_calc(&(PID_Trigger[1]), motor_shoot[2].speed_rpm, -500);
////                        break;
//  }
//      Shoot_cmd_CAN(PID_Shooter[0].out, PID_Shooter[1].out, PID_Trigger[1].out);
//      Shoot_cmd_CAN(PID_Shooter[0].out, PID_Shooter[1].out, 0);
//}
//void Shoot_Move_Choose(Shoot_Data_t *Shoot_Data_p)
//{
//if (Rc_ctrl.rc.s[1] == 1)
//      {
//          Shoot_Data_p->Shoot_Mode = SHOOT_SILENCE;
//          Shoot_Data.Shoot_RPM_Set[0] = 0;
//          Shoot_Data.Shoot_RPM_Set[1] = 0;
//      }
//      //����ģʽ�ж�
//      if (Rc_ctrl.rc.s[1] == 3)
//      {

//          Shoot_Data.Shoot_RPM_Set[0] = 5000;
//          Shoot_Data.Shoot_RPM_Set[1] = -5000;
//          if(Rc_ctrl.rc.ch[4] <= -100) Shoot_Data_p->Shoot_Mode = SHOOT_KEEP;
//          else Shoot_Data_p->Shoot_Mode = SHOOT_SILENCE;
//      }
//
//      //��������ģʽ�ж�
////        if (Rc_ctrl.rc.s[1] == 2)
////        {
////            Shoot_Data.Shoot_RPM_Set[0] = 6000;
////            Shoot_Data.Shoot_RPM_Set[1] = -6000;
////            if (Rc_ctrl.rc.ch[4] <= -300)
////            {
////                osDelay(20);
////                if (Rc_ctrl.rc.ch[4] >= -300) Shoot_Data_p->Shoot_Mode = SHOOT_ANGLE;
////            }
////
////        }
//      //�˵�ģʽ�ж�
//      if (Rc_ctrl.rc.ch[4] >= 300)
//      {
//          Shoot_Data_p->Shoot_Mode = SHOOT_ROLLBACK;
//      }
//}
//int8_t Shoot_Done_Check(Shoot_Data_t *Shoot_Data_p)
//{
//  if (motor_trigger.angle >=Shoot_Data_p->Trigger_Angle_Set-5 && motor_trigger.angle <= Shoot_Data_p->Trigger_Angle_Set+5 )
//      return 1;
//  else
//      return 0;
//}

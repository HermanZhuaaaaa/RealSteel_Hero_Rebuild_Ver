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
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{
    const float trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    const float shooter_speed_pid[3] = {SHOOTER_SPEED_PID_KP, SHOOTER_SPEED_PID_KI, SHOOTER_SPEED_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_RC = get_remote_control_point();
    //电机数据
    shoot_control.shoot_motor_measure[0] = get_shooter_motor_measure_point(0);
    shoot_control.shoot_motor_measure[1] = get_shooter_motor_measure_point(1);
    shoot_control.trigger_motor_measure = get_trigger_motor_measure_point();
    //PID初始化
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.shooter_motor_pid[0], PID_POSITION, shooter_speed_pid, SHOOTER_SPEED_PID_MAX_OUT, SHOOTER_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control.shooter_motor_pid[1], PID_POSITION, shooter_speed_pid, SHOOTER_SPEED_PID_MAX_OUT, SHOOTER_SPEED_PID_MAX_IOUT);
    //更新数据
    shoot_feedback_update();
    //官步的摩擦轮电机是通过PWM控制的
    //  不使用射击速度变换，直接全速射击
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
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
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
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
    }

    else if (shoot_control.shoot_mode ==SHOOT_READY_BULLET)
    {
        if (shoot_control.key == SWITCH_TRIGGER_OFF)
        {
            //设置拨弹轮的拨动速度,并开启堵转反转处理（英雄不能反转）
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
        //设置拨弹轮的速度
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
        //设置拨弹轮的拨动速度,并开启堵转反转处理（无反转）
        shoot_control.trigger_speed_set = CONTINUE_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else if (shoot_control.shoot_mode == SHOOT_DONE)
    {
        shoot_control.speed_set = 0.0f;
    }

    //这是啥意思
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.given_current = 0;
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        //ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        //ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        PID_calc(&shoot_control.shooter_motor_pid[0], shoot_control.shoot_motor_measure[0]->speed_rpm, SHOOTER_SPEED_RPM_SET);
        PID_calc(&shoot_control.shooter_motor_pid[1], shoot_control.shoot_motor_measure[1]->speed_rpm, -SHOOTER_SPEED_RPM_SET);
    }
    else
    {
        //计算拨弹轮电机PID
        PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);

        if (shoot_control.shoot_mode < SHOOT_READY_BULLET)
        {
            shoot_control.given_current = 0;
        }

        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
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
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
void shoot_set_mode(void)
{
    int8_t last_s = RC_SW_UP;

    //上拨判断，一次开启，再次关闭
    if ((switch_is_up(shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_up(shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //处于中档，使用键盘控制拨弹盘 （？）
    if (switch_is_mid(shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_RC->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    //处于中档， 可以使用键盘关闭摩擦轮
    else if (switch_is_mid(shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_RC->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //什么意思？
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
    //什么意思？

    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //下拨一次或者鼠标按下一次，进入射击状态
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
        //鼠标长按一直进入射击状态 保持连发
        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode =SHOOT_READY_BULLET;
        }
    }

    //来自裁判系统处理
    //get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);

    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_RC->rc.s[SHOOT_RC_MODE_CHANNEL];
}

/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
void shoot_feedback_update(void)
{
    float speed_filter_1 = 0.0f;
    float speed_filter_2 = 0.0f;
    float speed_filter_3 = 0.0f;
    //拨弹盘电机速度滤波，参数不懂
    const float filter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    //二阶低通滤波
    speed_filter_1 = speed_filter_2;
    speed_filter_2 = speed_filter_3;
    speed_filter_3 = speed_filter_2 * filter_num[0] + speed_filter_1 * filter_num[1] + (shoot_control.trigger_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * filter_num[2];
    shoot_control.speed = speed_filter_3;

    //电机圈数重置，因为输出轴旋转一圈，电机旋转36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
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

    //计算输出轴角度
    shoot_control.angle = (shoot_control.ecd_count*ECD_RANGE + shoot_control.trigger_motor_measure->ecd)*MOTOR_ECD_TO_ANGLE;
    //微动开关
    //啥jb东西
    //shoot_control.key = BUTTEN_TRIG_PIN;
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;

    //长按计时
    if (shoot_control.press_l)//左键如果按下
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)//就一直开始计数，计到400
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)//右键
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

    //射击开关下档时间计时
    //不打算使用遥控器射击，但可以保留 通过左侧拨杆判断 s[1]
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

    //官步的左键慢速射击，右键高速射击，英雄取消此设定，低速根本法射不动
    //不使用以下代码，等会看看怎么做
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
  * @brief          堵转倒转处理
  *                 英雄无法堵转掉头，故不使用此代码，无需处理
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
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
void shoot_bullet_control(void)
{
    //每次拨动60度，1/3PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.angle_set = rad_format(shoot_control.angle + PI_TEN);
        shoot_control.move_flag = 1;
    }

    if (shoot_control.key == SWITCH_TRIGGER_OFF)
    {
        shoot_control.shoot_mode = SHOOT_DONE;
    }

    //到达角度判断
    if (rad_format(shoot_control.angle_set - shoot_control.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
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
//      //TODO 单发射击
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
////        未完成、用不上
////        case SHOOT_AIM_AUTO :
////                        break;
//      case SHOOT_ANGLE:
//            PID_calc(&(PID_Trigger[0]), motor_trigger.angle, Shoot_Data.Trigger_Angle_Set);
//                      PID_calc(&(PID_Trigger[1]), motor_shoot[2].speed_rpm, PID_Trigger[0].out);
//                      break;
//      case SHOOT_KEEP:
//            PID_calc(&(PID_Trigger[1]), motor_shoot[2].speed_rpm, -600);
//                      break;
////        英雄是无法退蛋的
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
//      //连射模式判断
//      if (Rc_ctrl.rc.s[1] == 3)
//      {

//          Shoot_Data.Shoot_RPM_Set[0] = 5000;
//          Shoot_Data.Shoot_RPM_Set[1] = -5000;
//          if(Rc_ctrl.rc.ch[4] <= -100) Shoot_Data_p->Shoot_Mode = SHOOT_KEEP;
//          else Shoot_Data_p->Shoot_Mode = SHOOT_SILENCE;
//      }
//
//      //三发点射模式判断
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
//      //退弹模式判断
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

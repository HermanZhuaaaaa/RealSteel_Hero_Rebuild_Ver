#include "Shoot_task.h"
#include "Gimbal_perform.h"
#include "Motor.h"
shoot_control_t shoot_control;
extern Motor_dat_t motor_shoot[3];

void shoot_task_main(void *argument)
{
    /* USER CODE BEGIN shoot_task_main */

    vTaskDelay(SHOOT_TASK_INIT_TIME);
    shoot_init();

    /* Infinite loop */
    for (;;)
    {
		shoot_set_mode_2();
        shoot_control_loop();
		Shoot_cmd_CAN(shoot_control.shooter_motor_pid[0].out,shoot_control.shooter_motor_pid[1].out,shoot_control.trigger_motor_pid.out);
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
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, trigger_speed_pid, TRIGGER_BULLET_PID_MAX_OUT, TRIGGER_BULLET_PID_MAX_IOUT);
    PID_init(&shoot_control.shooter_motor_pid[0], PID_POSITION, shooter_speed_pid, SHOOTER_SPEED_PID_MAX_OUT, SHOOTER_SPEED_PID_MAX_IOUT);
    PID_init(&shoot_control.shooter_motor_pid[1], PID_POSITION, shooter_speed_pid, SHOOTER_SPEED_PID_MAX_OUT, SHOOTER_SPEED_PID_MAX_IOUT);
	
	shoot_control.Fric_speed_set[0] = 0;
	shoot_control.Fric_speed_set[1] = 0;
	
	shoot_control.trigger_speed_set = 0;
}


/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ����can����ֵ
  */
void shoot_control_loop(void)
{
	//Ħ���ֵ��pid����
    PID_calc(&shoot_control.shooter_motor_pid[0],motor_shoot[0].speed_rpm, shoot_control.Fric_speed_set[0]);
    PID_calc(&shoot_control.shooter_motor_pid[1],motor_shoot[1].speed_rpm, shoot_control.Fric_speed_set[1]);
	//�����̵���ջ�
	PID_calc(&shoot_control.trigger_motor_pid, motor_shoot[2].speed_rpm, shoot_control.trigger_speed_set);
}

void shoot_set_mode_2(void)
{
	//ң�����ϲ���Ħ���ֺͲ����̴�����ת��
	if(switch_is_up(shoot_control.shoot_RC->rc.s[1]))
	{
		shoot_control.Fric_speed_set[0] = 8500;
		shoot_control.Fric_speed_set[1] = -8500;
		if (shoot_control.shoot_RC->mouse.press_l)
		{
			shoot_control.trigger_speed_set = -250;
		}
		else
		{
			shoot_control.trigger_speed_set = 0;	
		}
		
	}
	//ң������������ģʽ���������
	else
	{
		shoot_control.Fric_speed_set[0] = 0;
		shoot_control.Fric_speed_set[1] = 0;
		shoot_control.trigger_speed_set = 0;
	}
}
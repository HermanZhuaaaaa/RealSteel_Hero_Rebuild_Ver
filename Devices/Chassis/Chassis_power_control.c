#include "Chassis_power_control.h"
#include "arm_math.h"
//#include "referee.h" ����ϵͳ

/**
  * @brief          ���ƹ��ʣ���Ҫ���Ƶ������
  * @param[in]      chassis_power_control: ��������
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
	float chassis_power = 0.0f;
	float chassis_power_buffer = 0.0f;
	float total_current_limit = 0.0f;
	float total_current = 0.0f;
	//��ȡ������ID��û����
	//uint8_t robot_id = get_robot_id();
	
}


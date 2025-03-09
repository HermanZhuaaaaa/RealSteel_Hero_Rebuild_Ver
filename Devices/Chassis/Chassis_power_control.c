#include "Chassis_power_control.h"
#include "arm_math.h"
//#include "referee.h" 裁判系统

/**
  * @brief          限制功率，主要限制电机电流
  * @param[in]      chassis_power_control: 底盘数据
  * @retval         none
  */
void chassis_power_control(chassis_move_t *chassis_power_control)
{
	float chassis_power = 0.0f;
	float chassis_power_buffer = 0.0f;
	float total_current_limit = 0.0f;
	float total_current = 0.0f;
	//获取机器人ID？没看懂
	//uint8_t robot_id = get_robot_id();
	
}


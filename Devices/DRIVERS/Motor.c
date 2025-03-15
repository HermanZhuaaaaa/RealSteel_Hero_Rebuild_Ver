#include "Motor.h"

// 底盘电机数据
Motor_dat_t motor_chassis[4];
//云台电机
Motor_dat_t motor_gimbal[2];
//拨弹盘、摩擦轮电机
Motor_dat_t motor_shoot[3];
//电机的角度信息 用于存储拨弹盘角度、Yaw角度、Pitch角度
//Motor_Angle motor_trigger_angle,motor_gimbal_pitch_angle,motor_gimbal_yaw_angle;

double chassis_power;
double current;
double voltage;

//fdcan1缓存区
uint8_t rx_data1[8] = {0};
void Fdcan1_callback(FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&fdcan_RxHeader,rx_data1) != HAL_OK);
	else fdcan_RxHeader.DataLength = fdcan_RxHeader.DataLength >> 16;
	
	switch(fdcan_RxHeader.Identifier)
	{
		case 0x201 : Motor_dat_transmit(&motor_chassis[0], rx_data1);
		             break;
		case 0x202 : Motor_dat_transmit(&motor_chassis[1], rx_data1);
		             break;
	    case 0x203 : Motor_dat_transmit(&motor_chassis[2], rx_data1);
		             break;
		case 0x204 : Motor_dat_transmit(&motor_chassis[3], rx_data1);
		             break;	
		case 0x212:
					voltage = ((int32_t)(rx_data1[1]<<8)|(int32_t)(rx_data1[0]))/100.0;
					current = ((int32_t)(rx_data1[3]<<8)|(int32_t)(rx_data1[2]))/100.0;
					chassis_power = voltage * current;		
					break;
		default : break;
	}
	
}

//fdcan2缓存
uint8_t rx_data2[8] = {0};	

//fdcan2回调函数
void Fdcan2_callback(FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_RxHeaderTypeDef fdcan_RxHeader;
    if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &fdcan_RxHeader, rx_data2) != HAL_OK);
	else fdcan_RxHeader.DataLength = fdcan_RxHeader.DataLength >> 16;
	
	switch(fdcan_RxHeader.Identifier)
	{
		case 0x201 : Motor_dat_transmit(&motor_shoot[0], rx_data2);
		             break;
		case 0x202 : Motor_dat_transmit(&motor_shoot[1], rx_data2);
		             break;
		case 0x203 : Motor_dat_transmit(&motor_shoot[2], rx_data2);
					//Cale_Trigger_angle_t(&motor_shoot[2], &motor_trigger_angle);
		             break;
		//Pitch
		case 0x206 : Motor_dat_transmit(&motor_gimbal[1], rx_data2);
					//Cale_Gimbal_angle_t(&motor_gimbal[1], &motor_gimbal_pitch_angle);
		             break;		
		//Yaw
		case 0x208 : Motor_dat_transmit(&motor_gimbal[0], rx_data2);
					//Cale_Gimbal_angle_t(&motor_gimbal[0], &motor_gimbal_yaw_angle);
		             break;

		default : break;
	}
}

/*计算Trigger圈数*/
void Cale_Trigger_angle_t(Motor_dat_t *motor,Motor_Angle *motor_data)
{

	if(motor->ecd-motor->last_ecd>4096) motor_data->round_cnt--;
	
	else if(motor->ecd-motor->last_ecd<-4096) motor_data->round_cnt++;
	else;
	
	motor_data->total_angle = motor_data->round_cnt * 8192 + motor->ecd - motor_data->offset_angle; //累计电机编码数，以上电时位置为0
	
	motor_data->angle = motor_data->total_angle/36.0/8192.0*360;
}

/*计算Gimbal圈数*/
void Cale_Gimbal_angle_t(Motor_dat_t *motor,Motor_Angle *motor_data)
{

	if(motor->ecd-motor->last_ecd>4096) motor_data->round_cnt--;
	
	else if(motor->ecd-motor->last_ecd<-4096) motor_data->round_cnt++;
	else;
	
	motor_data->total_angle = motor_data->round_cnt * 8192 + motor->ecd - motor_data->offset_angle; //累计电机编码数，以上电时位置为0
	
//	motor_data->angle = motor_data->total_angle/8192.0*360;
	motor_data->angle = motor->ecd/8192.0*360;
}

//对CAN反馈报文进行数据解析
void Motor_dat_transmit(Motor_dat_t *motor_t, uint8_t *Motor_dat)
{
	motor_t->last_ecd = motor_t->ecd;
	motor_t->ecd = Motor_dat[0] << 8 | Motor_dat[1];
	motor_t->speed_rpm = Motor_dat[2] << 8 | Motor_dat[3];
	motor_t->given_current = Motor_dat[4] << 8 | Motor_dat[5];
	motor_t->temperate = Motor_dat[6];
}

//通过FDCAN1发送CAN数据帧
//  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
//  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
//  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
//  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
//  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
void Chassis_cmd_CAN(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint8_t Dat[8];
	Dat[0] = motor1 >> 8;
	Dat[1] = motor1;
	Dat[2] = motor2 >> 8;
	Dat[3] = motor2;	
	Dat[4] = motor3 >> 8;
	Dat[5] = motor3;
	Dat[6] = motor4 >> 8;
	Dat[7] = motor4;
	
	fdcanx_send_data(&hfdcan1, CAN_CHASSIS_ALL_ID, Dat);

}
//通过fdcan2发送数据，控制YAW， PITCH两轴的M6020电机，其中输入变量
void Gimbal_cmd_CAN(int16_t motor1, int16_t motor2)
{
	uint8_t Dat_g[8];
	Dat_g[0] = 0;
	Dat_g[1] = 0;
	Dat_g[2] = motor2 >> 8;
	Dat_g[3] = motor2;
	Dat_g[4] = 0;
	Dat_g[5] = 0;
	Dat_g[6] = motor1 >> 8;
	Dat_g[7] = motor1;
	
	fdcanx_send_data(&hfdcan2, CAN_GIMBAL_ALL_ID, Dat_g);
}

//通过fdcan2发送数据，控制发射机构电机(两个摩擦轮电机、一个拨弹盘电机)，其中输入变量
void Shoot_cmd_CAN(int16_t motor1, int16_t motor2, int16_t motor3)
{
	uint8_t Dat_s[8];
	Dat_s[0] = motor1 >> 8;
	Dat_s[1] = motor1;
	Dat_s[2] = motor2 >> 8;
	Dat_s[3] = motor2;	
	Dat_s[4] = motor3 >> 8;
	Dat_s[5] = motor3;
	Dat_s[6] = 0;
	Dat_s[7] = 0;
	
	fdcanx_send_data(&hfdcan2, CAN_CHASSIS_ALL_ID, Dat_s);
}

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
Motor_dat_t *get_yaw_gimbal_motor_measure_point(void)
{
	return &motor_gimbal[0];
}


/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
Motor_dat_t *get_pitch_gimbal_motor_measure_point(void)
{
	return &motor_gimbal[1];
}

/**
  * @brief          返回拨弹电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
Motor_dat_t *get_shooter_motor_measure_point(uint8_t i)
{
	return &motor_shoot[i];
}

/**
  * @brief          返回拨弹电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
Motor_dat_t *get_trigger_motor_measure_point(void)
{
	return &motor_shoot[2];
}

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
Motor_dat_t *get_chassis_motor_measure_point(uint8_t i)
{
	return &motor_chassis[i];
}

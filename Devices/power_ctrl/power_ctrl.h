#ifndef POWER_CTRL_H
#define POWER_CTRL_H


#include "main.h"
#include "RLS.h"
#include "Chassis_Task.h"
#include "Referee_task.h"
#include "Motor.h"

#define ROBOTDEVISION HERO  //定义机器人类型（步兵英雄哨兵

const static float refereeFullBuffSet          = 60.0f;
const static float refereeBaseBuffSet          = 50.0f;
const static float capFullBuffSet              = 230.0f;
const static float capBaseBuffSet              = 30.0f;
const static float error_powerDistribution_set = 20.0f;
const static float prop_powerDistribution_set  = 15.0f;

const static float MAX_CAP_POWER_OUT                         = 300.0f;
const static float CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD = 43.0f;
const static float CAP_OFFLINE_ENERGY_TARGET_POWER           = 37.0f;
const static float MAX_POEWR_REFEREE_BUFF                    = 60.0f;
const static float REFEREE_GG_COE                            = 0.95f;
const static float CAP_REFEREE_BOTH_GG_COE                   = 0.85f;

const static uint8_t maxLevel                                     = 10U;
const static uint8_t HeroChassisPowerLimit_HP_FIRST[10]     = {55U, 60U, 65U, 70U, 75U, 80U, 85U, 90U, 100U, 120U};
const static uint8_t InfantryChassisPowerLimit_HP_FIRST[10] = {45U, 50U, 55U, 60U, 65U, 70U, 75U, 80U, 90U, 100U};
const static uint8_t SentryChassisPowerLimit                      = 100U;

typedef enum{
	INFANTRY = 1,
	HERO		 = 2,
	SENTRY   = 3,
} RobotDevision_e;

typedef enum{
	CapChargingMode  = 1,//电容充电模式
	CapDischargeMode = 2,//电容放电模式
} PowerMode_e;


typedef struct {
	double final_cmd;
	double Angel_v;
	double torque;
}Motor_Power_Modle_t;

typedef struct {
	
	FRLS_Estimator estimator;
	Motor_Power_Modle_t Motor[4];
	
	fp32 finalPowerLimited;				//最终功率限制
	fp32 sumPowerCmd_before_clamp;//处理前功率
	
	double k[2];	//电机模型参数						
	
	double sum_abs_Angel_v;       			// Σ|ω| 
	double sum_torque_squared; 					// Σ(τ)^2 
	double sum_Angelv_x_torquesquared;	// Σ(τ * ω)

	fp32 current_chassis_power;  //实时底盘功率
	
	pid_type_def powerPD_base;	 //baseBuff的PID控制结构体
	pid_type_def powerPD_full;	 //fullBuff的PID控制结构体
	
	fp32 fullBuffSet; //能量环上限的设定值（若超过能量上限，强制抬高maxPowerLimited）
	fp32 baseBuffSet;	//能量环下限的设定值（若低于能量下限，强制降低maxPowerLimited）
	
	fp32 fullMaxPower;//功率下限值（这里有一点反逻辑，近似于Pmax = P_referee-（Set-Ref），所以能量环与功率环设定值上下限相反
	fp32 baseMaxPower;//功率上限值
	
	fp32 powerBuff;		//超电剩余能量 或 裁判系统返回功率buff剩余能量
	
	uint8_t LATEST_FEEDBACK_ROBOT_LEVEL;//裁判系统最后一次返回的等级
	fp32 refereeMaxPower;								//裁判系统限制的最大取电功率
	fp32 powerUpperLimit; 							//绝对功率上限 
	fp32 MIN_MAXPOWER_CONFIGURED;  			//最小的最大功率限制（防止输出功率过低导致机动性变差 
	
	fp32 userConfiguredMaxPower;        //用于战术的用户自定义功率大小（受限于fullMaxPower和baseMaxPower（clamp函数实现取上下限
	
	fp32 newTorqueCurrent[4];						//最终can通信输出值
	
	//debug
	fp32 delta;
	
} Chassis_Power_Data_t;


void power_ctrl_init(Chassis_Power_Data_t* Chassis_Power_Data_p);
void power_ctrl(Chassis_Power_Data_t *Chassis_Power_Data_p);
void select_PowerMode(Chassis_Power_Data_t *Chassis_Power_Data_p, PowerMode_e PowerMode);


#endif

//typedef struct {
//	fp32 final_cmd[4];
//	fp32 Angel_v[4];
//	Motor_Angle motor_angle[4];
//	fp32 last_angle[4];
//}Motor_Power_Modle_t;


#ifndef POWER_CTRL_H
#define POWER_CTRL_H


#include "main.h"
#include "RLS.h"
#include "Chassis_Task.h"
#include "Referee_task.h"
#include "Motor.h"

#define ROBOTDEVISION HERO  //������������ͣ�����Ӣ���ڱ�

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
	CapChargingMode  = 1,//���ݳ��ģʽ
	CapDischargeMode = 2,//���ݷŵ�ģʽ
} PowerMode_e;


typedef struct {
	double final_cmd;
	double Angel_v;
	double torque;
}Motor_Power_Modle_t;

typedef struct {
	
	FRLS_Estimator estimator;
	Motor_Power_Modle_t Motor[4];
	
	fp32 finalPowerLimited;				//���չ�������
	fp32 sumPowerCmd_before_clamp;//����ǰ����
	
	double k[2];	//���ģ�Ͳ���						
	
	double sum_abs_Angel_v;       			// ��|��| 
	double sum_torque_squared; 					// ��(��)^2 
	double sum_Angelv_x_torquesquared;	// ��(�� * ��)

	fp32 current_chassis_power;  //ʵʱ���̹���
	
	pid_type_def powerPD_base;	 //baseBuff��PID���ƽṹ��
	pid_type_def powerPD_full;	 //fullBuff��PID���ƽṹ��
	
	fp32 fullBuffSet; //���������޵��趨ֵ���������������ޣ�ǿ��̧��maxPowerLimited��
	fp32 baseBuffSet;	//���������޵��趨ֵ���������������ޣ�ǿ�ƽ���maxPowerLimited��
	
	fp32 fullMaxPower;//��������ֵ��������һ�㷴�߼���������Pmax = P_referee-��Set-Ref���������������빦�ʻ��趨ֵ�������෴
	fp32 baseMaxPower;//��������ֵ
	
	fp32 powerBuff;		//����ʣ������ �� ����ϵͳ���ع���buffʣ������
	
	uint8_t LATEST_FEEDBACK_ROBOT_LEVEL;//����ϵͳ���һ�η��صĵȼ�
	fp32 refereeMaxPower;								//����ϵͳ���Ƶ����ȡ�繦��
	fp32 powerUpperLimit; 							//���Թ������� 
	fp32 MIN_MAXPOWER_CONFIGURED;  			//��С����������ƣ���ֹ������ʹ��͵��»����Ա�� 
	
	fp32 userConfiguredMaxPower;        //����ս�����û��Զ��幦�ʴ�С��������fullMaxPower��baseMaxPower��clamp����ʵ��ȡ������
	
	fp32 newTorqueCurrent[4];						//����canͨ�����ֵ
	
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


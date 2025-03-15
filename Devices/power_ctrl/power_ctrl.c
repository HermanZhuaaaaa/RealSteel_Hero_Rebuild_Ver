#include "power_ctrl.h"
#include "stdbool.h"
#define USE_AUTOPOWERCTRL__ 1 //使用功率控制
#define USE_SUPERCAP__ 0			//使用超级电容
#define UPDATE_K__ 1					//电机模型更新
#define OLD_ERFEREE 0					//旧版裁判系统（新版裁判系统不反馈底盘实时功率

#define BASIC_POWERLIMIT 45.0
//#define TORQUE_CONST 0.3*3591.0/178.0 //力矩常数 电机力矩常数*减速比 单位 N*m/A
#define TORQUE_CONST 0.3/3591.0*178
#define CAN2A_CONST 20.0/16384.0  //can报文到电机电流的映射
#define M_PI 3.1415926f

#if USE_SUPERCAP__
	extern SuperCap_RxData cap_rx_data;
	extern glut_TxData_t glut_TxData;
#endif

extern pid_type_def PID_Chassis_motor[4];
extern referee_info_t referee_info;
extern uint32_t SysTimerFreq;
extern uint32_t SuperCap_last_update_TimerCoun;
extern Motor_dat_t motor_chassis[4];
extern double chassis_power;

Chassis_Power_Data_t Chassis_Power_Data;

extern chassis_move_t chassis_move; 

static bool isRefereeConnected();
static bool isSuperCapConnected();
static void set_MaxPower_Configured(Chassis_Power_Data_t *Chassis_Power_Data_p, fp32 maxPower);
static inline float rpm2av(float rpm) { return rpm * M_PI / 30.0f; }//rpm转角速度
static inline float givencurrent2torque(float givencurrent) { return givencurrent * CAN2A_CONST * TORQUE_CONST; }
static inline double clamp(double x,double min,double max) { return fmax(fmin(x, max), min); }//钳制函数
static inline bool floatEqual(double a, double b) { return fabs(a - b) < 1e-5f; }//浮点数近似等于

void power_ctrl_init(Chassis_Power_Data_t* Chassis_Power_Data_p)
{
	Chassis_Power_Data_p->LATEST_FEEDBACK_ROBOT_LEVEL = 1u;    //初始化裁判系统上次更新等级
	Chassis_Power_Data_p->MIN_MAXPOWER_CONFIGURED			= 30.0f; //初始化最小的最大功率限制
	Chassis_Power_Data_p->powerUpperLimit 						= 45.0f; //初始化绝对最大功率限制
	Chassis_Power_Data_p->userConfiguredMaxPower			= 40.0f; //初始化自定义最大功率
	
	
	#if UPDATE_K__  //启用电机建模
		frls_init(&(Chassis_Power_Data_p->estimator), 0.99999, 1e-5);
		Chassis_Power_Data_p->estimator.theta[0] = 0.05504202407361;  //填入测得的数据
		Chassis_Power_Data_p->estimator.theta[1] = 0.01016429146635;
	#endif
	Chassis_Power_Data_p->k[0] = 0.0107f; //填入测得的数据
	Chassis_Power_Data_p->k[1] = 2.0;
	
	//初始化能量环PID
	fp32 powerPD_base_config[3] = {50.0f,0.0f,0.2f};
	fp32 powerPD_full_config[3] = {50.0f,0.0f,0.2f};
	PID_init(&(Chassis_Power_Data_p->powerPD_base),PID_POSITION,powerPD_base_config,300,0);
	PID_init(&(Chassis_Power_Data_p->powerPD_full),PID_POSITION,powerPD_full_config,300,0);
	
}


void power_ctrl(Chassis_Power_Data_t *Chassis_Power_Data_p)
{
	#if USE_AUTOPOWERCTRL__  //启用功率控制
	
		/*  检查外设连接  */
		volatile static bool Referee_is_Connected;
		Referee_is_Connected = isRefereeConnected();
		#if USE_SUPERCAP__ //启用超电
			volatile static bool SuperCap_is_Connected;
			SuperCap_is_Connected = (isSuperCapConnected() && (cap_rx_data.errorCode == 0));
		#endif
		
	
		/*  更新电机数据  */
		Chassis_Power_Data_p->sum_abs_Angel_v   				 = 0.0;
		Chassis_Power_Data_p->sum_torque_squared 				 = 0.0;
		Chassis_Power_Data_p->sum_Angelv_x_torquesquared = 0.0;
		for (int i = 0;i<4;i++)//更新角速度和力矩
		{
			//角速度
			Chassis_Power_Data_p->Motor[i].Angel_v = rpm2av(motor_chassis[i].speed_rpm);
			//Σ|ω|
			Chassis_Power_Data_p->sum_abs_Angel_v += fabs(Chassis_Power_Data_p->Motor[i].Angel_v);
			//力矩
			Chassis_Power_Data_p->Motor[i].torque = givencurrent2torque(motor_chassis[i].given_current);
			//Σ(τ)^2 
			Chassis_Power_Data_p->sum_torque_squared += Chassis_Power_Data_p->Motor[i].torque*Chassis_Power_Data_p->Motor[i].torque;
			//Σ(τ * ω)
			Chassis_Power_Data_p->sum_Angelv_x_torquesquared += Chassis_Power_Data_p->Motor[i].Angel_v * Chassis_Power_Data_p->Motor[i].torque;
		}
		
		
		/*  进行k的更新  */
		#if UPDATE_K__
			#if USE_SUPERCAP__
				if(SuperCap_is_Connected)
				{
					//获取实时功率
					Chassis_Power_Data_p->current_chassis_power = cap_rx_data.chassisPower;
					//模型训练
					frls_update(&(Chassis_Power_Data_p->estimator), 
												Chassis_Power_Data_p->sum_abs_Angel_v, 
												Chassis_Power_Data_p->sum_torque_squared, 
												Chassis_Power_Data_p->sum_Angelv_x_torquesquared, 
												Chassis_Power_Data_p->current_chassis_power);
					//赋值
					Chassis_Power_Data_p->k[0] = Chassis_Power_Data_p->estimator.theta[0];
					Chassis_Power_Data_p->k[1] = Chassis_Power_Data_p->estimator.theta[1];
				}
				
			#else
//				if(Referee_is_Connected) //只有当裁判系统连接且为旧版裁判系统时才进行模型更新
//				{
					if(Chassis_Power_Data_p->current_chassis_power != chassis_power)
					{
						Chassis_Power_Data_p->current_chassis_power = chassis_power;
						frls_update(&(Chassis_Power_Data_p->estimator), 
													Chassis_Power_Data_p->sum_abs_Angel_v, 
													Chassis_Power_Data_p->sum_torque_squared, 
													Chassis_Power_Data_p->sum_Angelv_x_torquesquared, 
													Chassis_Power_Data_p->current_chassis_power);
					
						Chassis_Power_Data_p->k[0] = Chassis_Power_Data_p->estimator.theta[0];
						Chassis_Power_Data_p->k[1] = Chassis_Power_Data_p->estimator.theta[1];
					}
//				}
			#endif /* USE_SUPERCAP__ */
		#endif /* UPDATE_K__ */
	
//				
//		/*  更新能量反馈数据  */
//		#if USE_SUPERCAP__
//			if (SuperCap_is_Connected)
//			{
//				Chassis_Power_Data_p->powerBuff = cap_rx_data.capEnergy;//优先使用电容数据
//			}
//			else if (Referee_is_Connected)
//			{
//				Chassis_Power_Data_p->powerBuff = referee_info.PowerHeatData.buffer_energy;
//			}
//			else
//			{
//				Chassis_Power_Data_p->powerBuff = 0;
//			}
//		#else
//			if (Referee_is_Connected)
//			{
//				Chassis_Power_Data_p->powerBuff = referee_info.PowerHeatData.buffer_energy;
//			}
//			else
//			{
//				Chassis_Power_Data_p->powerBuff = 0;
//			}
//		#endif /* USE_SUPERCAP__ */
//			
//			
//		/*  更新能量设定数据  */
//		#if USE_SUPERCAP__
//			if (SuperCap_is_Connected)
//			{
//				Chassis_Power_Data_p->fullBuffSet = capFullBuffSet;//看power_ctrl.h的常量定义
//        Chassis_Power_Data_p->baseBuffSet = capBaseBuffSet;
//			}
//			else
//			{
//				// 如果裁判系统也断联, 我们不开启能量环闭环, 所以我们不需要更新fullBuffSet和baseBuffSet，故没有对于Referee_is_Connected的判断
//				Chassis_Power_Data_p->fullBuffSet = refereeFullBuffSet;
//				Chassis_Power_Data_p->baseBuffSet = refereeBaseBuffSet;
//			}
//		#else
//				Chassis_Power_Data_p->fullBuffSet = refereeFullBuffSet;
//				Chassis_Power_Data_p->baseBuffSet = refereeBaseBuffSet;
//		#endif /* USE_SUPERCAP__ */
//		
//			
//		/*  更新系统绝对功率上限  */
//		#if USE_SUPERCAP__
//			if(Referee_is_Connected)
//			{
//				//获取裁判系统功率上限
//				Chassis_Power_Data_p->refereeMaxPower = fmax(referee_info.GameRobotState.chassis_power_limit, CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
//				//更新机器人等级
//				if (referee_info.GameRobotState.robot_level > 10U)
//						Chassis_Power_Data_p->LATEST_FEEDBACK_ROBOT_LEVEL = 1U;
//				else
//						Chassis_Power_Data_p->LATEST_FEEDBACK_ROBOT_LEVEL = fmax(1U, referee_info.GameRobotState.robot_level);
//				//更新绝对功率上限
//				if(SuperCap_is_Connected)
//						//绝对功率上限 = 裁判系统功率上限 + 电容硬件输出功率上限
//						Chassis_Power_Data_p->powerUpperLimit = Chassis_Power_Data_p->refereeMaxPower + MAX_CAP_POWER_OUT;
//				else
//						Chassis_Power_Data_p->powerUpperLimit = Chassis_Power_Data_p->refereeMaxPower + 50.0 * (sqrtf(refereeFullBuffSet) - sqrtf(refereeBaseBuffSet));
//			}
//			else
//			{
//				//通过最后的等级获取功率
//				switch (ROBOTDEVISION)
//				{
//					case INFANTRY:
//							Chassis_Power_Data_p->refereeMaxPower = InfantryChassisPowerLimit_HP_FIRST[Chassis_Power_Data_p->LATEST_FEEDBACK_ROBOT_LEVEL - 1U];
//							break;
//					case HERO:
//							Chassis_Power_Data_p->refereeMaxPower = HeroChassisPowerLimit_HP_FIRST[Chassis_Power_Data_p->LATEST_FEEDBACK_ROBOT_LEVEL - 1U];
//							break;
//          case SENTRY:
//							Chassis_Power_Data_p->refereeMaxPower = SentryChassisPowerLimit;
//							break;
//				}
//				if(SuperCap_is_Connected)
//						//电容断联依旧可以高功率输出
//						Chassis_Power_Data_p->powerUpperLimit = Chassis_Power_Data_p->refereeMaxPower + MAX_CAP_POWER_OUT;
//				else
//						//裁判系统 电容同时断联启用保守的功率输出
//						Chassis_Power_Data_p->powerUpperLimit = Chassis_Power_Data_p->refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
//			}
//			//设定最大功率输出的最小值
//			Chassis_Power_Data_p->MIN_MAXPOWER_CONFIGURED = Chassis_Power_Data_p->refereeMaxPower * 0.8f;
//		#else
//			if(Referee_is_Connected)
//			{
//				//获取裁判系统功率上限
//				Chassis_Power_Data_p->refereeMaxPower = fmax(referee_info.GameRobotState.chassis_power_limit, CAP_OFFLINE_ENERGY_RUNOUT_POWER_THRESHOLD);
//				//更新机器人等级
//				if (referee_info.GameRobotState.robot_level > 10U)
//						Chassis_Power_Data_p->LATEST_FEEDBACK_ROBOT_LEVEL = 1U;
//				else
//						Chassis_Power_Data_p->LATEST_FEEDBACK_ROBOT_LEVEL = fmax(1U, referee_info.GameRobotState.robot_level);
//				//更新绝对功率上限
//				Chassis_Power_Data_p->powerUpperLimit = Chassis_Power_Data_p->refereeMaxPower;
//			}
//			else
//			{
//				//通过最后的等级获取功率
//				switch (ROBOTDEVISION)
//				{
//					case INFANTRY:
//							Chassis_Power_Data_p->refereeMaxPower = InfantryChassisPowerLimit_HP_FIRST[Chassis_Power_Data_p->LATEST_FEEDBACK_ROBOT_LEVEL - 1U];
//							break;
//					case HERO:
//							Chassis_Power_Data_p->refereeMaxPower = HeroChassisPowerLimit_HP_FIRST[Chassis_Power_Data_p->LATEST_FEEDBACK_ROBOT_LEVEL - 1U];
//							break;
//          case SENTRY:
//							Chassis_Power_Data_p->refereeMaxPower = SentryChassisPowerLimit;
//							break;
//				}
//				//更新绝对功率上限
//				Chassis_Power_Data_p->powerUpperLimit = Chassis_Power_Data_p->refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
//			}
//			//设定最大功率输出的最小值
//			Chassis_Power_Data_p->MIN_MAXPOWER_CONFIGURED = Chassis_Power_Data_p->refereeMaxPower * 0.8f;
//		#endif /* USE_SUPERCAP__ */
//		
//			
//		/*  更新最大功率(计算能量环，能量环输出即为Pmax)  */
//		#if USE_SUPERCAP__ //启用超电
//		if (Referee_is_Connected || SuperCap_is_Connected)  //裁判系统 或 超电连接正常
//		#else
//		if (Referee_is_Connected)
//		#endif
//		{
//			Chassis_Power_Data_p->baseMaxPower = 
//					fmax((Chassis_Power_Data_p->refereeMaxPower - PID_calc(&(Chassis_Power_Data_p->powerPD_base), sqrtf(Chassis_Power_Data_p->powerBuff), sqrtf(Chassis_Power_Data_p->baseBuffSet))), Chassis_Power_Data_p->MIN_MAXPOWER_CONFIGURED);
//			Chassis_Power_Data_p->fullMaxPower = 
//					fmax((Chassis_Power_Data_p->refereeMaxPower - PID_calc(&(Chassis_Power_Data_p->powerPD_full), sqrtf(Chassis_Power_Data_p->powerBuff), sqrtf(Chassis_Power_Data_p->fullBuffSet))), Chassis_Power_Data_p->MIN_MAXPOWER_CONFIGURED);
//		}
//		else //全部断连
//		{
//			//全断联不开能量环
//			Chassis_Power_Data_p->baseMaxPower = Chassis_Power_Data_p->fullMaxPower = Chassis_Power_Data_p->refereeMaxPower * CAP_REFEREE_BOTH_GG_COE;
//			PID_clear(&(Chassis_Power_Data_p->powerPD_base));
//			PID_clear(&(Chassis_Power_Data_p->powerPD_full));
//		}
		
		 Chassis_Power_Data_p->baseMaxPower = Chassis_Power_Data_p->fullMaxPower = 55.0f;
		 
		
		/*  估算本次输出功率值  */
		volatile const float k0 = TORQUE_CONST * CAN2A_CONST; //扭矩输出比率 单位为 Nm/can报文电流映射
		
		static float newTorqueCurrent[4];
		
		float sumCmdPower = 0.0f;
    float cmdPower[4] = {0};

    float sumError = 0.0f;
    float error[4];
		
		Chassis_Power_Data_p->finalPowerLimited = clamp(Chassis_Power_Data_p->userConfiguredMaxPower,Chassis_Power_Data_p->fullMaxPower, Chassis_Power_Data_p->baseMaxPower);
		
		float allocatablePower = Chassis_Power_Data_p->finalPowerLimited;
    float sumPowerRequired = 0.0f;
		
		for (int i = 0; i < 4; i++)
		{
			cmdPower[i] = chassis_move.motor_speed_pid[i].out * k0 * Chassis_Power_Data_p->Motor[i].Angel_v + 
										fabs(Chassis_Power_Data_p->Motor[i].Angel_v) * Chassis_Power_Data_p->k[0] + 
										chassis_move.motor_speed_pid[i].out * k0 * chassis_move.motor_speed_pid[i].out * k0 * Chassis_Power_Data_p->k[1] +
										K3_CONST / 4.0;
		  sumCmdPower += cmdPower[i];
			error[i] = fabs(rpm2av(chassis_move.motor_speed_pid[i].set) - Chassis_Power_Data_p->Motor[i].Angel_v);
			if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
			{
					allocatablePower += -cmdPower[i];
			}
			else
			{
					sumError += error[i];
					sumPowerRequired += cmdPower[i];
			}
		}
		
		Chassis_Power_Data_p->sumPowerCmd_before_clamp = sumCmdPower;
		
		if (sumCmdPower > Chassis_Power_Data_p->finalPowerLimited)
		{
				float errorConfidence;
				//设置置信度
        if (sumError > error_powerDistribution_set)
        {
            errorConfidence = 1.0f;
        }
        else if (sumError > prop_powerDistribution_set)
        {
            errorConfidence =
                clamp((sumError - prop_powerDistribution_set) / (error_powerDistribution_set - prop_powerDistribution_set), 0.0f, 1.0f);
        }
        else
        {
            errorConfidence = 0.0f;
        }
				
				
				
				for (int i = 0; i < 4; i++)
				{
						if (floatEqual(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)
						{
								newTorqueCurrent[i] =  chassis_move.motor_speed_pid[i].out;
								continue;
						}
						//准备分配功率
						float powerWeight_Error = error[i] / sumError;
						float powerWeight_Prop  = cmdPower[i] / sumPowerRequired;
						float powerWeight       = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;
						//计算二次方程的delta=b^2-4ac
						float delta             = Chassis_Power_Data_p->Motor[i].Angel_v * Chassis_Power_Data_p->Motor[i].Angel_v -
																			4.0f * Chassis_Power_Data_p->k[1] * (Chassis_Power_Data_p->k[0] * fabs(Chassis_Power_Data_p->Motor[i].Angel_v) + K3_CONST / 4.0 - powerWeight * allocatablePower);
						
						Chassis_Power_Data_p->delta = delta;//debug
						
						//三种解求答案，利用单个电机功率反解扭矩
						if (floatEqual(delta, 0.0f))  // repeat roots
						{
								newTorqueCurrent[i] = -Chassis_Power_Data_p->Motor[i].Angel_v / (2.0f * Chassis_Power_Data_p->k[1]) / k0;
						}
						else if (delta > 0.0f)  // distinct roots
						{
								newTorqueCurrent[i] = chassis_move.motor_speed_pid[i].out > 0.0f ? (-Chassis_Power_Data_p->Motor[i].Angel_v + sqrtf(delta)) / (2.0f * Chassis_Power_Data_p->k[1]) / k0
																																			: (-Chassis_Power_Data_p->Motor[i].Angel_v - sqrtf(delta)) / (2.0f * Chassis_Power_Data_p->k[1]) / k0;
						}
						else  // imaginary roots
						{
								newTorqueCurrent[i] = -Chassis_Power_Data_p->Motor[i].Angel_v / (2.0f *  Chassis_Power_Data_p->k[1]) / k0;
						}
//						newTorqueCurrent[i] = fabs(round(newTorqueCurrent[i]) < chassis_move.motor_speed_pid[i].max_out ? newTorqueCurrent[i] : chassis_move.motor_speed_pid[i].max_out);
						if(fabs(newTorqueCurrent[i]) > chassis_move.motor_speed_pid[i].max_out)
						{
							if(newTorqueCurrent[i] >= 0)
								newTorqueCurrent[i] = chassis_move.motor_speed_pid[i].max_out;
							else
								newTorqueCurrent[i] = -chassis_move.motor_speed_pid[i].max_out;
						}
						Chassis_Power_Data_p->newTorqueCurrent[i] = newTorqueCurrent[i];
//						Chassis_Power_Data_p->newTorqueCurrent[i] = chassis_move.motor_speed_pid[i].out;
				}
				
		}
		else
		{
        for (int i = 0; i < 4; i++)
        {
						Chassis_Power_Data_p->newTorqueCurrent[i] = chassis_move.motor_speed_pid[i].out;
        }
		}
		#if USE_SUPERCAP__
		if(Chassis_Power_Data_p->finalPowerLimited < (Chassis_Power_Data_p->refereeMaxPower - 5))
		{
			glut_TxData.enable = 1;
			glut_TxData.charging = 1;
			glut_TxData.chargingPower = Chassis_Power_Data_p->refereeMaxPower - 5 - Chassis_Power_Data_p->finalPowerLimited;
			glut_TxData.feedbackRefereePowerLimit = Chassis_Power_Data_p->refereeMaxPower;
			glut_SuperCap_SendData();
		}
		else if(Chassis_Power_Data_p->finalPowerLimited > Chassis_Power_Data_p->refereeMaxPower)
		{
			glut_TxData.enable = 1;
			glut_TxData.charging = 0;
			glut_TxData.chargingPower = 0;
			glut_TxData.feedbackRefereePowerLimit = Chassis_Power_Data_p->refereeMaxPower;
			glut_SuperCap_SendData();
		}
		else
		{
			glut_TxData.enable = 1;
			glut_TxData.charging = 0;
			glut_TxData.chargingPower = 0;
			glut_TxData.feedbackRefereePowerLimit = Chassis_Power_Data_p->refereeMaxPower;
			glut_SuperCap_SendData();
		}
		#endif
		
	#else //不启用功率控制
		for(int i = 0;i<4;i++) Chassis_Power_Data_p->newTorqueCurrent[i] = chassis_move.motor_speed_pid[i].out;
	#endif /* USE_AUTOPOWERCTRL__ */
}


//设定自定义功率上限
static void set_MaxPower_Configured(Chassis_Power_Data_t *Chassis_Power_Data_p, fp32 maxPower)
{
	 Chassis_Power_Data_p->userConfiguredMaxPower = clamp(maxPower, Chassis_Power_Data_p->MIN_MAXPOWER_CONFIGURED, Chassis_Power_Data_p->powerUpperLimit);
}


//定义功率策略
void select_PowerMode(Chassis_Power_Data_t *Chassis_Power_Data_p, PowerMode_e PowerMode)
{
	if (PowerMode == CapChargingMode)
	{
		set_MaxPower_Configured(Chassis_Power_Data_p, Chassis_Power_Data_p->refereeMaxPower * 0.95f);
	}
	else if (PowerMode == CapDischargeMode)
	{
		set_MaxPower_Configured(Chassis_Power_Data_p, Chassis_Power_Data_p->refereeMaxPower + 35.0f);
	}
}


static bool isRefereeConnected()
{
	if (referee_info.last_update_TimerCount == 0) return false;
	static uint32_t Referee_delat_time;
	Referee_delat_time = osKernelGetSysTimerCount()-referee_info.last_update_TimerCount;
	if(Referee_delat_time<0)
		Referee_delat_time += 0xffffffff;
	if (Referee_delat_time < SysTimerFreq) return true;//超时不超过一秒
	else return false;
}

static bool isSuperCapConnected()
{
	if (SuperCap_last_update_TimerCoun == 0) return false;
	static uint32_t SuperCap_delat_time;
	SuperCap_delat_time = osKernelGetSysTimerCount()-SuperCap_last_update_TimerCoun;
	if(SuperCap_delat_time<0)
		SuperCap_delat_time += 0xffffffff;
	if (SuperCap_delat_time < SysTimerFreq) return true;//超时不超过一秒
	else return false;
}




//		static fp32 delat_time_s;
//		//计算时间差
//		delat_time_s = (osKernelGetSysTimerCount()-Chassis_Power_Data_p->last_update_count)/(SysTimerFreq*1.0);
//		Chassis_Power_Data_p->last_update_count = osKernelGetSysTimerCount();
//			Chassis_Power_Data_p->Motor.last_angle[i] = Chassis_Power_Data_p->Motor.motor_angle[i].angle;
//			Cale_Chassis_angle_t(&motor_chassis[i], &(Chassis_Power_Data_p->Motor.motor_angle[i]));
//			Chassis_Power_Data_p->Motor.Angel_v[i] = (Chassis_Power_Data_p->Motor.motor_angle[i].angle - Chassis_Power_Data_p->Motor.last_angle[i])/delat_time_s;
			
//				else if(Referee_is_Connected && OLD_ERFEREE)//电容断联时，只有当裁判系统连接且为旧版裁判系统时才进行模型更新
//				{
//					Chassis_Power_Data_p->current_chassis_power = referee_info.PowerHeatData.chassis_power;
//					
//					frls_update(&(Chassis_Power_Data_p->estimator), 
//												Chassis_Power_Data_p->sum_abs_Angel_v, 
//												Chassis_Power_Data_p->sum_torque_squared, 
//												Chassis_Power_Data_p->sum_Angelv_x_torquesquared, 
//												Chassis_Power_Data_p->current_chassis_power);
//					
//					Chassis_Power_Data_p->k[0] = Chassis_Power_Data_p->estimator.theta[0];
//				  Chassis_Power_Data_p->k[1] = Chassis_Power_Data_p->estimator.theta[1];
//				}



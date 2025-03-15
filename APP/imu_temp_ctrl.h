#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "pid.h"
#include "MahonyAHRS.h"


void IMU_task(void * argument);
void INS_Init(void);
void IMU_Temperature_Ctrl();
void INS_Task(void);

#endif 

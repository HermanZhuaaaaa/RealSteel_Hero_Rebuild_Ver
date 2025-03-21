#include "main.h"
#include "imu_temp_ctrl.h"

#define cheat TRUE  //作弊模式 去掉较小的gyro值   TRUE FALSE
#define correct_Time_define 1000    //上电去0飘 1000次取平均
#define temp_times 100       //探测温度阈值


IMU_Data_t IMU_Data;                //imu数据结构体

pid_type_def Temperature_PID= {0};          //温度pid
float Temperature_PID_Para[3]= {286, 0, 0}; //温度pid参数
float H723_Temperature=0.0f;                //内部温度
unsigned int adc_v;                         //电压值
float adcx;                                 //电压值
float RefTemp = 40;                         //目标温度
float tempcount = 1;                        //0飘初始化前温度

uint8_t attitude_flag=0;            //0飘初始化状态 0->未初始化 1->初始化中 2->初始化完成
uint32_t correct_times=0;           //0飘初始化次数
static uint8_t first_mahony=0;      //0飘初始化标志
uint32_t temp_temperature=0;        //0飘初始化后的温度
float gyro_correct[3]= {0};         //0飘初始值


//float gyro[3], accel[3], temp;        //陀螺仪原始值
//float roll,pitch,yaw=0;                       //欧拉角

void IMU_task(void  * argument)
{
    vTaskDelay(1000);
    INS_Init();

    /* Infinite loop */
    for (;;)
    {
        INS_Task();
        osDelay(1);
    }
}


void INS_Init(void)
{

    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f, 0); //ekf初始化
    PID_init(&Temperature_PID, PID_POSITION, Temperature_PID_Para, 2000, 0); //加热pid
    Mahony_Init(1000);  //mahony姿态解算初始化

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   //开启pwm,加热

    while (BMI088_init()); //陀螺仪初始化
}


void IMU_Temperature_Ctrl()
{
    PID_calc(&Temperature_PID, IMU_Data.temp, RefTemp); //温度pid  //需要调一下pid使得温度在47°左右
    temp_temperature=(uint32_t)Temperature_PID.out;

    if (Temperature_PID.out<0)
    {
        temp_temperature=0;
    }

    htim3.Instance->CCR4 = temp_temperature;
}

/***
 * @brief: INS_TASK(void const * argument)
 * @param: argument - 任务参数
 * @retval: void
 * @details: IMU姿态控制任务函数

*/

void INS_Task(void)  //1khz
{
    static uint32_t count = 0;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_read(IMU_Data.gyro, IMU_Data.accel, &(IMU_Data.temp)); //陀螺仪值读取

        if (first_mahony==0)
        {
            first_mahony++;
            MahonyAHRSinit(IMU_Data.accel[0], IMU_Data.accel[1], IMU_Data.accel[2], 0, 0, 0); //mahony上电快速初始化
        }

        if (attitude_flag==2) //ekf的姿态解算
        {
            IMU_Data.gyro[0]-=gyro_correct[0];   //减去陀螺仪0飘
            IMU_Data.gyro[1]-=gyro_correct[1];
            IMU_Data.gyro[2]-=gyro_correct[2];

#if cheat              //作弊 可以让yaw很稳定 去掉比较小的值0.003f); 0.02->yaw:0°/h;0.01->yaw:5°/h

            if (fabsf(IMU_Data.gyro[2])<0.012f)
                IMU_Data.gyro[2]=0;

#endif
            //===========================================================================
            //ekf姿态解算部分
            //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
            IMU_QuaternionEKF_Update(IMU_Data.gyro[0], IMU_Data.gyro[1], IMU_Data.gyro[2], IMU_Data.accel[0], IMU_Data.accel[1], IMU_Data.accel[2]);
            //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
            //===============================================================================

            //=================================================================================
            //mahony姿态解算部分
            //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
            Mahony_update(IMU_Data.gyro[0], IMU_Data.gyro[1], IMU_Data.gyro[2], IMU_Data.accel[0], IMU_Data.accel[1], IMU_Data.accel[2], 0, 0, 0);
            Mahony_computeAngles(); //角度计算
            //HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
            //=============================================================================
            //ekf获取姿态角度函数
            IMU_Data.pitch=Get_Pitch()+180; //获得pitch
            IMU_Data.roll=Get_Roll();//获得roll
            IMU_Data.yaw=Get_Yaw()+180;//获得yaw
            //==============================================================================
        }
        else if (attitude_flag==1)  //状态1 开始1000次的陀螺仪0飘初始化
        {
            //gyro correct
            gyro_correct[0]+=   IMU_Data.gyro[0];
            gyro_correct[1]+=   IMU_Data.gyro[1];
            gyro_correct[2]+=   IMU_Data.gyro[2];
            correct_times++;

            if (correct_times>=correct_Time_define)
            {
                gyro_correct[0]/=correct_Time_define;
                gyro_correct[1]/=correct_Time_define;
                gyro_correct[2]/=correct_Time_define;
                attitude_flag=2; //go to 2 state
            }
        }
    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz 的温度控制pid
        IMU_Temperature_Ctrl();
        //==================================================================================
        //单片机内部温度测量
        //                      HAL_ADC_Start(&hadc3);
        //                      adc_v = HAL_ADC_GetValue(&hadc3);
        adcx = (110.0-30.0)/(*(unsigned short*)(0x1FF1E840) - *(unsigned short*)(0x1FF1E820));
        H723_Temperature = adcx*(adc_v - *(unsigned short*)(0x1FF1E820))+30;
        //========================================================================================
        static uint32_t temp_Ticks=0;

        if ((fabsf(IMU_Data.temp-RefTemp)<5)&&attitude_flag==0) //接近额定温度之差小于5° 开始计数
        {
            temp_Ticks++;

            if (temp_Ticks>temp_times)  //计数达到一定次数后 才进入0飘初始化 说明温度已经达到目标
            {
                attitude_flag=1;  //go to correct state
                tempcount=IMU_Data.temp;
            }
        }
    }

    count++;
}


/**
************************************************************************
* @brief:       HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:      void
* @details:     GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ACC_INT_Pin)
    {
    }
    else if (GPIO_Pin == GYRO_INT_Pin)
    {

    }
}

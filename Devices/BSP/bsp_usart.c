#include "bsp_usart.h"
#include "main.h"
#include "string.h"
#include "remoter.h"

extern DMA_HandleTypeDef hdma_uart5_rx;

uint8_t usart5_buf[USART5_BUFLEN];
RC_ctrl_t RC_ctrl;


//开启遥控器不定长接收
void remoter_start(void)
{
	//HAL_UART_Receive_DMA(&huart5,usart5_buf,USART5_BUFLEN);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5,usart5_buf,USART5_BUFLEN);
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);		   // 手动关闭DMA_IT_HT中断
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	Dbus_to_rc(usart5_buf, &RC_ctrl);
//}


/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
void Dbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left

	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	if (rc_ctrl->rc.s[0] == 2) rc_ctrl->input_mode = RC_MouseKey;	//右侧拨杆在中间时为键鼠模式
	else if (rc_ctrl->rc.s[0] != 2) rc_ctrl->input_mode = RC_Remote;	//右侧拨杆在中间时为键鼠模式
}



/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &RC_ctrl;
}
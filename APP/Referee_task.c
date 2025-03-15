#include "Referee_task.h"
#include "string.h"
#include "crc_ref.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
#include "dma.h"
#include "bsp_usart.h"
#include "remoter.h"
#include "ui.h"

int cnt = 0;
referee_info_t referee_info;              // ����ϵͳ����
uint8_t uart7_rx_buffer[255];
extern DMA_HandleTypeDef hdma_uart7_rx;

uint32_t SysTimerFreq;

extern uint8_t usart5_buf[USART5_BUFLEN];

extern RC_ctrl_t RC_ctrl;
extern referee_ui_data_t referee_ui_data;
int ui_self_id = 0;							//������ID

void referee_task_main(void *argument)
{
	SysTimerFreq = osKernelGetSysTimerFreq();
    /* USER CODE BEGIN referee_task_main */
    //HAL_UARTEx_ReceiveToIdle_DMA(&huart5,usart5_buf,USART5_BUFLEN);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_buffer, 255);
    __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);           // �ֶ��ر�DMA_IT_HT�ж�
	referee_ui_init();


    //vTaskDelay(1000);
    /* Infinite loop */
    for (;;)
    {
        cnt ++;
		referee_ui_test_app(cnt);
        referee_ui_change(&referee_ui_data);

        vTaskDelay(100);
    }

    /* USER CODE END referee_task_main */
}


/**
 * @brief  ��ȡ��������,�ж��ж�ȡ��֤�ٶ�
 * @param  buff: ��ȡ���Ĳ���ϵͳԭʼ����
 * @retval �Ƿ�������ж�������
 * @attention  �ڴ��ж�֡ͷ��CRCУ��,������д�����ݣ����ظ��ж�֡ͷ
 */
void referee_read_data(uint8_t *buff)
{
    uint16_t judge_length; // ͳ��һ֡���ݳ���

    if (buff == NULL)      // �����ݰ��������κδ���
    {
        return;
    }

    // д��֡ͷ����(5-byte),�����ж��Ƿ�ʼ�洢��������
    memcpy(&referee_info.FrameHeader, buff, LEN_HEADER);

    // �ж�֡ͷ����(0)�Ƿ�Ϊ0xA5
    if (buff[SOF] == REFEREE_SOF)
    {
        // ֡ͷCRC8У��
        if (Verify_CRC8_Check_Sum(buff, LEN_HEADER) == TRUE)
        {
            // ͳ��һ֡���ݳ���(byte),����CR16У��
            judge_length = buff[DATA_LENGTH] + LEN_HEADER + LEN_CMDID + LEN_TAIL;

            // ֡βCRC16У��
            if (Verify_CRC16_Check_Sum(buff, judge_length) == TRUE)
            {
                // 2��8λƴ��16λint
                referee_info.CmdID = (buff[6] << 8 | buff[5]);

                // ��������������,�����ݿ�������Ӧ�ṹ����(ע�⿽�����ݵĳ���)
                // ��8���ֽڿ�ʼ�������� data=7
                switch (referee_info.CmdID)
                {
                case ID_game_state: // 0x0001
                    memcpy(&referee_info.GameState, (buff + DATA_Offset), LEN_game_state);
                    break;

                case ID_game_result: // 0x0002
                    memcpy(&referee_info.GameResult, (buff + DATA_Offset), LEN_game_result);
                    break;

                case ID_game_robot_survivors: // 0x0003
                    memcpy(&referee_info.GameRobotHP, (buff + DATA_Offset), LEN_game_robot_HP);
                    break;

                case ID_event_data: // 0x0101
                    memcpy(&referee_info.EventData, (buff + DATA_Offset), LEN_event_data);
                    break;

                case ID_supply_projectile_action: // 0x0102
                    memcpy(&referee_info.SupplyProjectileAction, (buff + DATA_Offset), LEN_supply_projectile_action);
                    break;

                case ID_game_robot_state: // 0x0201
                    memcpy(&referee_info.GameRobotState, (buff + DATA_Offset), LEN_game_robot_state);
                    break;

                case ID_power_heat_data: // 0x0202
                    memcpy(&referee_info.PowerHeatData, (buff + DATA_Offset), LEN_power_heat_data);
                    break;

                case ID_game_robot_pos: // 0x0203
                    memcpy(&referee_info.GameRobotPos, (buff + DATA_Offset), LEN_game_robot_pos);
                    break;

                case ID_buff_musk: // 0x0204
                    memcpy(&referee_info.BuffMusk, (buff + DATA_Offset), LEN_buff_musk);
                    break;

                case ID_aerial_robot_energy: // 0x0205
                    memcpy(&referee_info.AerialRobotEnergy, (buff + DATA_Offset), LEN_aerial_robot_energy);
                    break;

                case ID_robot_hurt: // 0x0206
                    memcpy(&referee_info.RobotHurt, (buff + DATA_Offset), LEN_robot_hurt);
                    break;

                case ID_shoot_data: // 0x0207
                    memcpy(&referee_info.ShootData, (buff + DATA_Offset), LEN_shoot_data);
                    break;

                case ID_student_interactive: // 0x0301   syhtodo���մ���δ����
                    memcpy(&referee_info.ReceiveData, (buff + DATA_Offset), LEN_receive_data);
                    break;
                }
            }
        }

        referee_info.last_update_TimerCount = osKernelGetSysTimerCount();

        // �׵�ַ��֡����,ָ��CRC16��һ�ֽ�,�����ж��Ƿ�Ϊ0xA5,�Ӷ��ж�һ�����ݰ��Ƿ��ж�֡����
        if (*(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL) == 0xA5)
        {
            // ���һ�����ݰ������˶�֡����,���ٴε��ý�������,ֱ���������ݰ��������
            referee_read_data(buff + sizeof(xFrameHeader) + LEN_CMDID + referee_info.FrameHeader.DataLength + LEN_TAIL);
        }
    }
}

/**
 * @brief ����ϵͳ���ݷ��ͺ���
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len)
{
    HAL_UART_Transmit_DMA(&huart7, send, tx_len);
    osDelay(115);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART7)
    {
        referee_read_data(uart7_rx_buffer);
		ui_self_id = referee_info.GameRobotState.robot_id;
        memset(uart7_rx_buffer, 0, 255);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_buffer, 255);
        __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);         // �ֶ��ر�DMA_IT_HT�ж�
    }
    if (huart->Instance == UART5)
    {
        Dbus_to_rc(usart5_buf, &RC_ctrl);
        remoter_start();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
}

#ifndef __REFEREE_TASK_H__
#define __REFEREE_TASK_H__

#include "referee_protocol.h"


/**
 * @brief ����ϵͳ���ݷ��ͺ���
 * @param
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

void referee_read_data(uint8_t *buff);

#endif

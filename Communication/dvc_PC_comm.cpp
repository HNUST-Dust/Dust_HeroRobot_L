/**
 * @file dvc_PC_comm.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* Includes ------------------------------------------------------------------*/

#include "dvc_PC_comm.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/**
 * @brief PcComm初始化函数
 * 
 */
void PcComm::Init()
{
    dwt_init(168);

    static const osThreadAttr_t KPcCommTaskAttr = 
    {
        .name = "pccomm_task",
        .stack_size = 256,
        .priority = (osPriority_t) osPriorityNormal
    };
    // 启动任务，将 this 传入
    osThreadNew(PcComm::TaskEntry, this, &KPcCommTaskAttr);
}

/**
 * @brief 任务入口（静态函数）—— osThreadNew 需要这个原型
 * 
 * @param argument 
 */
void PcComm::TaskEntry(void *argument)
{
    PcComm *self = static_cast<PcComm *>(argument);
    self->Task();
}

/**
 * @brief PcComm更新自瞄数据函数
 * 
 */
void PcComm::UpdataAutoaimData()
{
    send_autoaim_data.start_of_frame = 0xA5;
    send_autoaim_data.current_yaw    = INS.Yaw;
    send_autoaim_data.current_pitch  = -INS.Roll;
    send_autoaim_data.state          = 1;
    send_autoaim_data.enemy_color    = 1;
    send_autoaim_data.reserved       = 0;
}


/**
 * @brief PcComm发送信息函数
 * 
 */
void PcComm::Send_Message()
{
    uint16_t lenth = sizeof(send_autoaim_data);
    uint8_t buffer[lenth];

    send_autoaim_data.crc16 = 0;

    memcpy(buffer, &send_autoaim_data, lenth);
    append_crc16_check_sum(buffer, lenth);

    usb_transmit(buffer, lenth);
}

/**
 * @brief PcComm任务函数
 * 
 */
void PcComm::Task()
{
    for(;;)
    {
        UpdataAutoaimData();
        Send_Message();
        osDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief PcComm接收回调函数
 * 
 */
void PcComm::RxCpltCallback()
{
    if(bsp_usb_rx_buffer[0] == 0xA5)
    {
        uint16_t lenth = sizeof(recv_autoaim_data);
        memcpy(&recv_autoaim_data, bsp_usb_rx_buffer, lenth);
    }
}

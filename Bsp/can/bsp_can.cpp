/**
 * @file drv_can.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的CAN通信初始化与配置流程
 * @version 0.1
 * @date 2022-08-02
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_can.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

struct Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
struct Struct_CAN_Manage_Object CAN2_Manage_Object = {0};
struct Struct_CAN_Manage_Object CAN3_Manage_Object = {0};

// CAN通信发送缓冲区
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x2ff_Tx_Data[8];
uint8_t CAN1_0x3fe_Tx_Data[8];
uint8_t CAN1_0x4fe_Tx_Data[8];

uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x2ff_Tx_Data[8];
uint8_t CAN2_0x3fe_Tx_Data[8];
uint8_t CAN2_0x4fe_Tx_Data[8];

uint8_t CAN_Supercap_Tx_Data[8];

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief 初始化CAN总线
 *
 * @param hcan CAN编号
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(FDCAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    if (hcan->Instance == FDCAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(1) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == FDCAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(14) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(15) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }
    else if (hcan->Instance == FDCAN3)
    {
        CAN3_Manage_Object.CAN_Handler = hcan;
        CAN3_Manage_Object.Callback_Function = Callback_Function;
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(28) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
        CAN_Filter_Mask_Config(hcan, CAN_FILTER(29) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0, 0);
    }

    HAL_FDCAN_Start(hcan);

    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    HAL_FDCAN_ActivateNotification(hcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
}

/**
 * @brief 配置CAN的过滤器
 *
 * @param hcan CAN编号
 * @param Object_Para 编号 | FIFOx | ID类型 | 帧类型
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x3ff, 0x1fffffff)
 */
void CAN_Filter_Mask_Config(FDCAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    FDCAN_FilterTypeDef filter = {0};

    assert_param(hcan != NULL);

    // 解析参数
    uint8_t filter_index = Object_Para >> 3;
    uint8_t fifo_select  = (Object_Para >> 2) & 0x01;
    uint8_t id_type_flag = (Object_Para >> 1) & 0x01;
    uint8_t frame_type   = Object_Para & 0x01;

    filter.FilterIndex   = filter_index;
    filter.IdType        = id_type_flag ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterType    = FDCAN_FILTER_MASK;
    filter.FilterConfig  = (fifo_select == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    filter.FilterID1     = ID;
    filter.FilterID2     = Mask_ID;
    UNUSED(frame_type); // 帧类型在FDCAN中不需要配置

    HAL_FDCAN_ConfigFilter(hcan, &filter);
}

/**
 * @brief 发送数据帧
 *
 * @param hcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(FDCAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    //检测传参是否正确
    assert_param(hcan != NULL);

    tx_header.Identifier            = ID;
    tx_header.IdType                = FDCAN_STANDARD_ID;            // 标准ID
    tx_header.TxFrameType           = FDCAN_DATA_FRAME;             // 数据帧
    tx_header.DataLength            = Length;                       // 数据长度，注意有些HAL可能用DLC编码，需要确认
    tx_header.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;             // 错误指示，默认正常
    tx_header.BitRateSwitch         = FDCAN_BRS_OFF;                // 不启用比特率切换
    tx_header.FDFormat              = FDCAN_CLASSIC_CAN;            // 经典CAN格式
    tx_header.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;           // 不启用事件FIFO
    tx_header.MessageMarker         = 0;                            // 消息标记为0
    UNUSED(used_mailbox); // 避免未使用变量警告
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &tx_header, Data);
}
void CAN_PeriodElapsedCallback()
{
    // DJI电机专属
    // CAN1电机

}
/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &CAN1_Manage_Object.Rx_Buffer.Header,
                                   CAN1_Manage_Object.Rx_Buffer.Data);

            CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
        }
        else if (hfdcan->Instance == FDCAN2)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &CAN2_Manage_Object.Rx_Buffer.Header,
                                   CAN2_Manage_Object.Rx_Buffer.Data);

            CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
        }
        else if (hfdcan->Instance == FDCAN3)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &CAN3_Manage_Object.Rx_Buffer.Header,
                                   CAN3_Manage_Object.Rx_Buffer.Data);

            CAN3_Manage_Object.Callback_Function(&CAN3_Manage_Object.Rx_Buffer);
        }
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET)
    {
        if (hfdcan->Instance == FDCAN1)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1,
                                   &CAN1_Manage_Object.Rx_Buffer.Header,
                                   CAN1_Manage_Object.Rx_Buffer.Data);

            CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
        }
        else if (hfdcan->Instance == FDCAN2)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1,
                                   &CAN2_Manage_Object.Rx_Buffer.Header,
                                   CAN2_Manage_Object.Rx_Buffer.Data);

            CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
        }
        else if (hfdcan->Instance == FDCAN3)
        {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1,
                                   &CAN3_Manage_Object.Rx_Buffer.Header,
                                   CAN3_Manage_Object.Rx_Buffer.Data);

            CAN3_Manage_Object.Callback_Function(&CAN3_Manage_Object.Rx_Buffer);
        }
    }
}


/************************ COPYRIGHT(C) HNUST-DUST **************************/

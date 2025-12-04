#include "debug_tools.h"
#include <cstdint>
#include <cstring>
#include "usart.h"
#include "bsp_usart.h"

void DebugTools::VofaSendFloat(float data)
{
    uint8_t data_buf[4];
    memcpy(data_buf, &data, sizeof(float));
    // uart_send_data(&huart7, data_buf, 4);
    HAL_UART_Transmit(&huart7, data_buf, 4*sizeof(uint8_t), HAL_MAX_DELAY);
}

void DebugTools::VofaSendTail()
{
    uint8_t tail[4] = {0x00,0x00,0x80,0x7f};
    // uart_send_data(&huart7, tail, 4);
    HAL_UART_Transmit(&huart7, tail, 4*sizeof(uint8_t), HAL_MAX_DELAY);

}
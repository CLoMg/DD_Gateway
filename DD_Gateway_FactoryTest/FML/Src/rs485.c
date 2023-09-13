/*----------------------------------include-----------------------------------*/
#include "rs485.h"
#include "usart.h"
#include "shell_port.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

RS485_HandleTypeDef dev_rs485[] ={
    {
        &huart3,
        GPIOB,
        GPIO_PIN_2
    }
};
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void RS485_Init(RS485_HandleTypeDef *rs485) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能UART和GPIO时钟
    //__HAL_RCC_UARTX_CLK_ENABLE(); // 替换为实际的UART时钟
    __HAL_RCC_GPIOB_CLK_ENABLE(); // 替换为实际的GPIO时钟
    
    // 配置GPIO引脚
    GPIO_InitStruct.Pin = rs485->de_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(rs485->de_port, &GPIO_InitStruct);
    
    // 初始化UART
    // rs485->huart.Instance = USART3;
    // rs485->huart.Init.BaudRate = 115200;
    // rs485->huart.Init.WordLength = UART_WORDLENGTH_8B;
    // rs485->huart.Init.StopBits = UART_STOPBITS_1;
    // rs485->huart.Init.Parity = UART_PARITY_NONE;
    // rs485->huart.Init.Mode = UART_MODE_TX_RX;
    // rs485->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    // rs485->huart.Init.OverSampling = UART_OVERSAMPLING_16;
    // rs485->huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    // rs485->huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    // HAL_UART_Init(rs485->huart);
    
    // 初始化DE引脚为发送模式
    HAL_GPIO_WritePin(rs485->de_port, rs485->de_pin, GPIO_PIN_RESET);
}

void RS485_SendData(RS485_HandleTypeDef *rs485, uint8_t *data, uint16_t size) {
    // 切换到发送模式
    HAL_GPIO_WritePin(rs485->de_port, rs485->de_pin, GPIO_PIN_SET);
    
    // 发送数据
    HAL_UART_Transmit(rs485->huart, data, size, HAL_MAX_DELAY);
    
    // 切换到接收模式
    HAL_GPIO_WritePin(rs485->de_port, rs485->de_pin, GPIO_PIN_RESET);
}

void RS485_ReceiveData(RS485_HandleTypeDef *rs485, uint8_t *data, uint16_t size) {
    // 切换到接收模式
    HAL_GPIO_WritePin(rs485->de_port, rs485->de_pin, GPIO_PIN_RESET);
    
    // 接收数据
    HAL_UART_Receive(rs485->huart, data, size, HAL_MAX_DELAY);
}
/*------------------------------------test------------------------------------*/
/**
 * @brief RS485测试函数
 *        使用RS485发送数据，通过接收器查看是否成功接收
 * 
 * @param str 
 */
void RS485_Test(char *str)
{
  RS485_SendData(&dev_rs485[0], (uint8_t *)str, strlen(str));
}

SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
rs485_send, RS485_Test, rs485 send test);
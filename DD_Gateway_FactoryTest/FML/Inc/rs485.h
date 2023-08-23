#ifndef RS485_H
#define RS485_H

#include "main.h"

// 定义RS485结构体，包含UART端口和引脚信息
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *de_port; // DE控制引脚所在的GPIO
    uint16_t de_pin; // DE控制引脚
} RS485_HandleTypeDef;

extern RS485_HandleTypeDef dev_rs485[];
// 初始化RS485
void RS485_Init(RS485_HandleTypeDef *rs485);

// 发送数据
void RS485_SendData(RS485_HandleTypeDef *rs485, uint8_t *data, uint16_t size);

// 接收数据
void RS485_ReceiveData(RS485_HandleTypeDef *rs485, uint8_t *data, uint16_t size);

#endif /* RS485_DRIVER_H */

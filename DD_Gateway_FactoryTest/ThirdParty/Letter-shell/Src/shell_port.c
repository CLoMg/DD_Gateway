/**
 * @brief shell移植到STM32L431时的接口实现
 * @author mculover666
 * @date 2020/03/27 
*/

#include "shell.h"
#include <stm32l4xx_hal.h>
#include "rs485.h"
#include "usart.h"
#include "shell_port.h"

/* 1. 创建shell对象，开辟shell缓冲区 */
Shell shell;
char shell_buffer[200];
char test_ok_cnt = 0;
char test_error_cnt = 0;


/* 2. 自己实现shell写函数 */

//shell写函数原型：typedef void (*shellWrite)(const char);
 signed short User_Shell_Write(char *ch, unsigned short len)
{   
    signed short re_code = 0;
    //调用STM32 HAL库 API 使用查询方式发送
    //while(len--)
       //re_code = HAL_UART_Transmit(&huart3, (uint8_t *)(ch++), 1, 0xFFFF);
    RS485_SendData(&dev_rs485[0], (uint8_t *)ch, len);
    return re_code; 
}

/* 3. 编写初始化函数 */
void User_Shell_Init(void)
{
 //注册自己实现的写函数
    shell.write = User_Shell_Write;
 
 //调用shell初始化函数
    shellInit(&shell, shell_buffer, 200);
}
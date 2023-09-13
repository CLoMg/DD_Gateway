/**
 ******************************************************************************
 *
 * @file   
 *
 * @brief   
 *
 * @attention
 *
 *Copyright (c) 2023 STMicroelectronics.
 *
 *All rights reserved.
 ******************************************************************************
 */


/*----------------------------------include-----------------------------------*/
#include "ATGM332D.h"
#include "usart.h"
#include "shell_port.h"
#include "stdarg.h"
#include "string.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------variable----------------------------------*/
int bds_dev = -1;
uint8_t dev0_buff[200]={0xff,};

/*----------------------------------typedef-----------------------------------*/
ATGM332D_HandleTypeDef atgm_dev[]=
{
    {
        "uart2/bds_0",
        &huart2,
        GPIOA,
        GPIO_PIN_2,
        dev0_buff,
        0,
        sizeof(dev0_buff)/sizeof(uint8_t)
    }
};
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void atgm_io_init(ATGM332D_HandleTypeDef *device)
{
     GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能UART和GPIO时钟
    //__HAL_RCC_UARTX_CLK_ENABLE(); // 替换为实际的UART时钟
    __HAL_RCC_GPIOA_CLK_ENABLE(); // 替换为实际的GPIO时钟
    
    // 配置GPIO引脚
    GPIO_InitStruct.Pin = device->en_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(device->en_port, &GPIO_InitStruct);
    
    
    // 初始化DE引脚为发送模式
    HAL_GPIO_WritePin(device->en_port, device->en_pin, GPIO_PIN_RESET);
}
int atgm_open(char *dev_name)
{
    uint8_t i = 0;
    for(i = 0;i < sizeof(atgm_dev)/ sizeof(ATGM332D_HandleTypeDef);i++)
    {
        if(strcmp(dev_name,atgm_dev[i].dev_name) == 0)
        {
            atgm_io_init(&atgm_dev[i]);
            return i;
        }
    }
    return -1;
};
int atgm_close(int fd);
int atgm_write(int fd);
int atgm_read(int fd,uint8_t *buff,...){
    if(fd >= sizeof(atgm_dev)/ sizeof(ATGM332D_HandleTypeDef))
        return -1;
    else{
        int cmd;
        va_list p_args;
        va_start(p_args,buff);
        cmd = va_arg(p_args,int);
        va_end(p_args);
        if(cmd == 0){
            memcpy(buff,(uint8_t *)atgm_dev[fd].data_buff,atgm_dev[fd].data_index);
            return atgm_dev[fd].data_index;
        }
    }
};
int atgm_ioctl(int fd);
void atgm_handler(UART_HandleTypeDef *huart,uint8_t *data,uint16_t len)
{
    uint8_t i = 0;
    //shellPrint(&shell,"%s",data);
    for(i = 0;i < sizeof(atgm_dev)/ sizeof(ATGM332D_HandleTypeDef);i++)
    {
        if(huart == atgm_dev[i].huart)
        {
            // if(len > atgm_dev[i].capacity)
            // {
            //     atgm_dev[i].capacity = sizeof(atgm_dev->data_buff)/sizeof(uint8_t);
            //     atgm_dev[i].data_index = 0;
            // }
            if(len >300)
                len =300;
            memcpy(atgm_dev[i].data_buff,data,len);
            // atgm_dev[i].capacity -= len;
            atgm_dev[i].data_index = len;
        }
    }
}
/*------------------------------------test------------------------------------*/
/**
 * @brief BDS测试函数
 *        打印BDS信息
 * 
 * @param cmd 
 * @param ... 
 */
void BDS_Test(char *cmd,...)
{
  int param_1 = -1;
  uint16_t len;
  uint8_t buff[300];

  if(strcmp(cmd,"read") == 0)
  {
    va_list p_args;
    va_start(p_args,cmd);
    param_1 = va_arg(p_args,int);
    va_end(p_args);
    len  = atgm_read(bds_dev,buff,param_1);

    if(len > 0)
      shellPrint(&shell,buff,len);
    memset(buff,0,300);
  }
}
SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
bds_test, BDS_Test, bde_dev test);
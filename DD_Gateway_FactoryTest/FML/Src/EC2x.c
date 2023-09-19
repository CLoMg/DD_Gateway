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
#include "EC2x.h"
#include "usart.h"
#include "timer.h"
#include "shell_port.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fsm.h"
#include "queue.h"
/*-----------------------------------macro------------------------------------*/
#define BUFF_LEN 200
/*----------------------------------typedef-----------------------------------*/


/*----------------------------------variable----------------------------------*/
uint8_t ec2x_buff[BUFF_LEN]={0xff,};

//定义tcp连接字符串
uint8_t tcp_connect[]="AT+QIOPEN=1,0,\"TCP\",\"115.236.153.170\",52894,0,2\r\n";

//定义tcp连接指令队列
uint8_t tcpconnct_step = 0;
ATMsg_TypeDef connectmsg_queue[] = {
    {"AT\r\n","OK",200,5},
    {"AT+CPIN?\r\n","READY",200,5},
    {"AT+CREG?\r\n","+CREG: 0,1", 200, 5},
    {"AT+CGREG?\r\n","+CGREG: 0,1", 200, 5},
    {"AT+QICSGP=1,1,\"CMNET\",\"\",\"\",1\r\n","OK", 200, 5},
    {"AT+QIDEACT=1\r\n", "OK", 200, 5},
    {"AT+QIACT=1\r\n", "OK", 200, 5},
    {tcp_connect, "CONNECT", 500, 5},
};

static uint8_t reset_times = 0;

/*----------------------------------typedef-----------------------------------*/

EC2x_HandleTypeDef ec2x_dev[]=
{
    {
        "uart1/ec200s",
        &huart1,
        GPIOA,
        GPIO_PIN_12,
        GPIOA,
        GPIO_PIN_8,
        GPIOA,
        GPIO_PIN_11,
        {
            0,
           ec2x_buff,
            0,
            0,
            0,
            BUFF_LEN 
        }
    }
};
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

/**
 * @brief 设备初始化函数
 * 
 */
void ec2x_io_init(EC2x_HandleTypeDef *device)
{
     GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能UART和GPIO时钟
    //__HAL_RCC_UARTX_CLK_ENABLE(); // 替换为实际的UART时钟
    __HAL_RCC_GPIOA_CLK_ENABLE(); // 替换为实际的GPIO时钟
    
    // 配置GPIO引脚
    GPIO_InitStruct.Pin = device->reset_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(device->reset_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = device->wake_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(device->wake_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = device->pwren_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(device->pwren_port, &GPIO_InitStruct);
    
    
    // 初始化复位引脚为高电平
    HAL_GPIO_WritePin(device->reset_port, device->reset_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(device->wake_port, device->wake_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(device->pwren_port, device->pwren_pin, GPIO_PIN_SET);
    Delay_ms(1000);
    HAL_GPIO_WritePin(device->pwren_port, device->pwren_pin, GPIO_PIN_RESET);
}
/**
 * @brief 通过POWEREN引脚，对模块进行掉电复位
 * 
 * @param fd 
 * @return int 
 */
int ec2x_reset(int fd){
    fd = 0;
    int err_code = 0;
    if(fd >= sizeof(ec2x_dev)/ sizeof(EC2x_HandleTypeDef))
        return (err_code = -1);
    else{
        EC2x_HandleTypeDef *device = &ec2x_dev[fd];
        HAL_GPIO_WritePin(device->pwren_port, device->pwren_pin, GPIO_PIN_SET);
        Delay_ms(500);
        HAL_GPIO_WritePin(device->pwren_port, device->pwren_pin, GPIO_PIN_RESET);
    }
    reset_times++;
    return(ec2x_cmd_send(fd,NULL,0,"RDY\r\n",20000));
}

/**
 * @brief 设备文件open函数
 * 
 * @param dev_name 
 * @return int 
 *         返回设备文件索引，未查找到则返回-1
 */
int ec2x_open(char *dev_name)
{
    uint8_t i = 0;
    for(i = 0;i < sizeof(ec2x_dev)/ sizeof(EC2x_HandleTypeDef);i++)
    {
        if(strcmp(dev_name,ec2x_dev[i].dev_name) == 0)
        {
            ec2x_io_init(&ec2x_dev[i]);
            if(ec2x_reset(i))
                shellPrint(&shell,"ec2x initial OK\r\n");
            else
                shellPrint(&shell,"ec2x initial not Failed\r\n");
            return i;
        }
    }
    return -1;
};

/**
 * @brief 设备文件close函数
 * 
 * @param fd 
 * @return int 
 */
int ec2x_close(int fd);

/**
 * @brief 设备文件写入函数
 * 
 * @param fd 
 * @param tx_buff 
 * @param len 
 * @return int 
 *         设备号无效返回-1
 *         写入失败返回 0
 *          写入成功返回1
 *         
 */
int ec2x_write(int fd,uint8_t *tx_buff,uint16_t len){
    int err_code = 0;
    if(fd >= sizeof(ec2x_dev)/ sizeof(EC2x_HandleTypeDef))
        return (err_code = -1);
    else{
        err_code =  HAL_UART_Transmit_IT(ec2x_dev[fd].huart, tx_buff,len);
        while(__HAL_UART_GET_FLAG(ec2x_dev[fd].huart,UART_FLAG_TC) != SET);
        return(!err_code);
    }
};
/**
 * @brief 设备文件读取函数
 *        读取环形缓冲区指定长度数据
 * 
 * @param fd 
 * @param rx_buff 
 * @param len 
 * @return int 
 *         -1 接口错误
 *          0 读取失败
 *          n 读取成功字节数
 */
int ec2x_read(int fd,uint8_t *rx_buff,uint16_t len){
    uint8_t *dest = rx_buff;
    if(fd >= sizeof(ec2x_dev)/ sizeof(EC2x_HandleTypeDef))
        return -1;
    else{
        uint16_t j = 0;
        
        RingBuffer_TypeDef *src = &ec2x_dev[fd].msg;
        //如果队列被上锁了，则等待100ms
        if(src->block == 1)
        {
            Delay_ms(100);
            if(src->block == 1)
                return 0;
        }
        else{
            src->block = 1;
            if(len > src->size)
                len = src->size;
            for(j = 0 ; j < len; j++){
                dest[j] = src->data[src->head];
                src->head = (src->head + 1) % src->capacity;
                src->size--;
            }
            src->block = 0;
        }
        return len;
    }
};
int ec2x_ioctl(int fd);

/**
 * @brief 将串口接收数据copy到环形缓冲区，并更新缓冲区状态
 * 
 * @param huart 
 * @param data 
 * @param len 
 */
void ec2x_handler(UART_HandleTypeDef *huart,uint8_t *data,uint16_t len)
{
    uint8_t i = 0,j = 0;
    shellPrint(&shell,"%s",data);
    for(i = 0;i < sizeof(ec2x_dev)/ sizeof(EC2x_HandleTypeDef);i++)
    {
        if(huart == ec2x_dev[i].huart)
        {
            RingBuffer_TypeDef *dest = &ec2x_dev[i].msg;
            if(dest->block  == 0){
                dest->block = 1;
                for(j = 0 ; j < len; j++){
                   
                    dest->data[dest->tail] = data[j];
                    dest->tail = (dest->tail +1) % dest->capacity;
                    dest->size++;
                    dest->size = dest->size > dest->capacity ? dest->capacity : dest->size;
                }
                if((dest->tail > dest->head)&&(dest->size == dest->capacity))
                    dest->head = dest->tail;
                dest->block = 0;
            }
        }
    }
}


/**
 * @brief 回复匹配函数
 *        1）匹配成功，则插入回复成功事件
 *        2.1）匹配失败，超时次数-1
 *        2.2）匹配失败，超时次数=0，插入超时事件消息
 * 
 * @param expect 
 * @return int 
 */
int ec2x_replymatch(uint8_t* expect){
    uint8_t rx_buff[200]  = {0x00};
    int bytes_num = 0;
    bytes_num =  ec2x_read(0,rx_buff,200);
    if(bytes_num > 0)
    {
        if(strstr((const char*)rx_buff,(const char*)expect) != NULL){
            return 1;
        }
    }
    return 0;
}


/**
 * @brief ec2x指令发送函数
 *        1）发送指令
 *        2）进入等待回复状态
 * @param fd 
 * @param tx_buff 
 * @param len 
 * @param expect_reply 
 * @param timeout 
 */
int ec2x_cmd_send(int fd,uint8_t *tx_buff,uint16_t len,uint8_t *expect_reply,uint16_t timeout)
{
    if(tx_buff !=NULL){
        ec2x_dev[fd].msg.block = 1;
        ec2x_dev[fd].msg.head = 0;
        ec2x_dev[fd].msg.tail = 0;
        ec2x_dev[fd].msg.size = 0;
        ec2x_dev[fd].msg.capacity = BUFF_LEN;
        memset(ec2x_dev[fd].msg.data,0x00,BUFF_LEN);
        ec2x_dev[fd].msg.block = 0;
        ec2x_write(fd,tx_buff,len);
    }
    if(expect_reply != NULL){
        uint16_t retry_times = timeout / 100;
        while(retry_times--){
            Delay_ms(100);
            if(ec2x_replymatch(expect_reply) == 1)
                return  1;
        }
    }
    return 0;
}

/**
 * @brief tcp 连接流程
 * 
 * @param fd 
 * @return uint8_t 
 */
uint8_t ec2x_tcp_connect(int fd){
    static uint8_t step = 0,retry =0;
    reset_times = 0;
    if(fd >= sizeof(ec2x_dev)/ sizeof(EC2x_HandleTypeDef)){
        shellPrint(&shell,"no invalid ec2x device sn\r\n");
        return -1;
    }
    while(step < 9){
        if(reset_times > 5){
            shellPrint(&shell,"TCP Connect Failed\r\n");
            return 0;
        }
        switch (step)
        {
            case 0: 
            {
                if(ec2x_cmd_send(fd,connectmsg_queue[step].txmsg,strlen(connectmsg_queue[step].txmsg),connectmsg_queue[step].expect_reply,connectmsg_queue->timeout))
                {    
                    step++;
                    retry = 0;
                }
                else
                {
                    if(connectmsg_queue[step].retry_times < retry)
                    {
                        shellPrint(&shell,"%s err\r\n",connectmsg_queue[step].txmsg);
                        ec2x_reset(fd); 
                        step = 0;
                    }
                    else{
                        retry++;
                        Delay_ms(200);
                    }
                }
                Delay_ms(100);
            }
            break;
            case 1:
            {
                if(ec2x_cmd_send(fd,connectmsg_queue[step].txmsg,strlen(connectmsg_queue[step].txmsg),connectmsg_queue[step].expect_reply,connectmsg_queue->timeout))
                {    
                    step++;
                    retry = 0;
                }
                else
                {
                    if(connectmsg_queue[step].retry_times < retry){
                        shellPrint(&shell,"%s err\r\n",connectmsg_queue[step].txmsg);
                        ec2x_reset(fd); 
                        step = 0;
                    }
                    else{
                        retry++;
                        Delay_ms(200);
                    }
                }
                Delay_ms(100);
            }
            break;
            case 2:
            {
                if(ec2x_cmd_send(fd,connectmsg_queue[step].txmsg,strlen(connectmsg_queue[step].txmsg),connectmsg_queue[step].expect_reply,connectmsg_queue->timeout))
                {    
                    step++;
                    retry = 0;
                }
                else
                {
                    if(connectmsg_queue[step].retry_times < retry){
                        shellPrint(&shell,"%s err\r\n",connectmsg_queue[step].txmsg);
                        ec2x_reset(fd); 
                        step = 0;
                    }

                    else{
                        retry++;
                        Delay_ms(200);
                    }
                }
                Delay_ms(100);
            }
            break;
            case 3:
            {
                if(ec2x_cmd_send(fd,connectmsg_queue[step].txmsg,strlen(connectmsg_queue[step].txmsg),connectmsg_queue[step].expect_reply,connectmsg_queue->timeout))
                {    
                    step += 2;
                    retry = 0;
                }
                else
                {
                    if(connectmsg_queue[step].retry_times < retry){
                        shellPrint(&shell,"%s err\r\n",connectmsg_queue[step].txmsg);
                        ec2x_reset(fd); 
                        step = 0;
                    }

                    else{
                        retry++;
                        Delay_ms(200);
                    }
                }
                Delay_ms(100);                
            }
            break;
            case 4:
            {
                if(ec2x_cmd_send(fd,connectmsg_queue[step].txmsg,strlen(connectmsg_queue[step].txmsg),connectmsg_queue[step].expect_reply,connectmsg_queue->timeout))
                {    
                    step++;
                    retry = 0;
                }
                else
                {
                    if(connectmsg_queue[step].retry_times < retry){
                        shellPrint(&shell,"%s err\r\n",connectmsg_queue[step].txmsg);
                        ec2x_reset(fd); 
                        step = 0;
                    }
                    else{
                        retry++;
                        Delay_ms(200);
                    }
                }
                Delay_ms(100);               
            }
            break;
            case 5:
            {
                if(ec2x_cmd_send(fd,connectmsg_queue[step].txmsg,strlen(connectmsg_queue[step].txmsg),connectmsg_queue[step].expect_reply,connectmsg_queue->timeout))
                {    
                    step++;
                    retry = 0;
                }
                else
                {
                    if(connectmsg_queue[step].retry_times < retry){
                        shellPrint(&shell,"%s err\r\n",connectmsg_queue[step].txmsg);
                        ec2x_reset(fd); 
                        step = 0;
                    }
                    else{
                        retry++;
                        Delay_ms(200);
                    }
                }
                Delay_ms(100);                
            }
            break; 
            case 6:
            {
                if(ec2x_cmd_send(fd,connectmsg_queue[step].txmsg,strlen(connectmsg_queue[step].txmsg),connectmsg_queue[step].expect_reply,connectmsg_queue->timeout))
                {    
                    step++;
                    retry = 0;
                }
                else
                {
                    if(connectmsg_queue[step].retry_times < retry){
                        shellPrint(&shell,"%s err\r\n",connectmsg_queue[step].txmsg);
                        ec2x_reset(fd); 
                        step = 0;
                    }
                    else{
                        retry++;
                        Delay_ms(200);
                    }
                }
                Delay_ms(100);
            }
            break; 
            case 7:
            {
                if(ec2x_cmd_send(fd,connectmsg_queue[step].txmsg,strlen(connectmsg_queue[step].txmsg),connectmsg_queue[step].expect_reply,connectmsg_queue->timeout))
                {    
                    step++;
                    retry = 0;
                }
                else
                {
                    if(connectmsg_queue[step].retry_times < retry){
                        shellPrint(&shell,"%s err\r\n",connectmsg_queue[step].txmsg);
                        ec2x_reset(fd); 
                        step = 0;
                    }
                    else{
                        retry++;
                        Delay_ms(200);
                    }
                }
                Delay_ms(100);
            }
            break;  
            case 8:
            {
                shellPrint(&shell,"tcp is connected\r\n");
                return 1;
            }
            break; 
            default:
                break;
        }
    }
}
/*------------------------------------test------------------------------------*/

/**
 * @brief 
 * 
 * @param fd 
 * @param tx_buff 
 * @param expect_reply 
 * @param timeout 
 */
void EC2x_Test(int fd,char *tx_buff,char *expect_reply,uint16_t timeout)
{
    uint8_t tx_data[20] = {0xff,};
    uint8_t reply_data[20] = {0xff,};
    uint8_t tx_len = 0,reply_len = 0;
    tx_len = strlen(tx_buff);
    reply_len = strlen(expect_reply);
    if(strcmp(tx_buff,"tcp_connect")==0){
        ec2x_tcp_connect(fd);
    }
    else{
        memcpy(tx_data,tx_buff,tx_len);
        tx_data[tx_len] = 0x0D;
        tx_data[tx_len+1] = 0x0A;
        tx_len += 2;
        memcpy(reply_data,expect_reply,reply_len);

        if(ec2x_cmd_send(fd,tx_data,tx_len,reply_data,timeout)){
            shellPrint(&shell,"EC2x CMD Test OK\r\n");
            test_ok_cnt++;
        }
        else
            shellPrint(&shell,"EC2x CMD Test Failed\r\n");
    }
}

SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
ec2x_test, EC2x_Test, send command and wait for reply);

SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
ec2x_connect, ec2x_tcp_connect, ec2x tcp connect);
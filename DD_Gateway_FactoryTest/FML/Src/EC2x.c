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
#include "fsm.h"
#include "queue.h"
/*-----------------------------------macro------------------------------------*/
#define BUFF_LEN 300
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
uint8_t ec2x_buff[BUFF_LEN]={0xff,};

FSM_T *ec2x_fsm = NULL;

StateTransform_T ec2x_transtable[] = 
{
	{UINIT,Timeout_Event,UINIT,ec2x_reset},
	{UINIT,ReplyScs_Event,AT_MODE,NULL},
	{AT_MODE,CMDSend_Event,WAIT_REPLY,NULL},
	{WAIT_REPLY,ReplyScs_Event,AT_MODE,NULL},
	{WAIT_REPLY,Timeout_Event,AT_MODE,NULL},
	{TRANS_MODE,MsgSend_Event,TRANS_MODE,NULL},
};

uint8_t event_buff[10];
Queue_HandleTypeDef ec2x_event_queue;


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
    HAL_Delay(1000);
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
        HAL_Delay(500);
        HAL_GPIO_WritePin(device->pwren_port, device->pwren_pin, GPIO_PIN_RESET);
    }
    ec2x_waitreply("\r\nRDY",10000);
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

             queue_init(&ec2x_event_queue,event_buff,10);
            // ec2x_fsm = (FSM_T*)malloc(sizeof(FSM_T));
            // uint8_t num = sizeof(ec2x_transtable)/sizeof(ec2x_transtable[0]);
            // FSM_Regist(ec2x_fsm,ec2x_transtable,num,UINIT);
            // queue_insert(&ec2x_event_queue,Timeout_Event);
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
            HAL_Delay(100);
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

static uint8_t timeout_cnt = 0;
void ec2x_timeout_handle(void){
    shellPrint(&shell,"no valid data\r\n");
    queue_insert(&ec2x_event_queue,Timeout_Event);
}

int ec2x_replymatch(char* expect){
    uint8_t rx_buff[300]  = {0x00};
    uint8_t evt = 0;
    int bytes_num = 0;
    bytes_num =  ec2x_read(0,rx_buff,300);
    if(bytes_num > 0)
    {
        if(strstr(rx_buff,expect) != NULL){
             shellPrint(&shell,"recv success");
             evt = ReplyScs_Event;
             queue_insert(&ec2x_event_queue,evt);
            return 1;
        }
    }
    
    if(timeout_cnt == 0)
         ec2x_timeout_handle(); 
    else
        timeout_cnt--;
    return 0;
}

void ec2x_cmd_send(int fd,uint8_t *tx_buff,uint16_t len,uint8_t *expect_reply,uint16_t timeout)
{
    ec2x_write(fd,tx_buff,len);
    ec2x_waitreply(expect_reply,timeout);
}

/***/
void ec2x_waitreply(uint8_t *expect,uint16_t timeout){
    timeout_cnt = timeout / 100;
    timer_insert(100,timeout_cnt,ec2x_replymatch,expect);
}


void ec2x_fms_proccess(void){
    if(queue_is_empty(&ec2x_event_queue) == 0){
        EC2x_Event_T evt = 0x00;
        queue_pop(&ec2x_event_queue,&evt,1);
        FSM_EventHandle(ec2x_fsm,evt);
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
    uint8_t *tx_data,len=0;
    len = strlen(tx_buff);
    tx_data = (char *)malloc((len+2)*sizeof(uint8_t));
    memcpy(tx_data,tx_buff,len);
    tx_data[len] = 0x0D;
    tx_data[len+1] = 0x0A;
    ec2x_cmd_send(fd, tx_data,len+2, expect_reply,timeout);
    free(tx_data);
    tx_data = NULL;
}

SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
ec2x_cmd_send, EC2x_Test, ec2x test);


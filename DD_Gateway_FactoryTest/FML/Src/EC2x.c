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
#define BUFF_LEN 200
/*----------------------------------typedef-----------------------------------*/
void ec2x_queue_send(void);
void ec2x_reply_handler(void);
void ex2x_timeout_handler(void);
/*----------------------------------variable----------------------------------*/
uint8_t ec2x_buff[BUFF_LEN]={0xff,};

FSM_T ec2x_fsm;

StateTransform_T ec2x_transtable[] = 
{
	{UINIT,Timeout_Event,UINIT,ec2x_reset},
	{UINIT,ReplyScs_Event,AT_MODE,NULL},
	{AT_MODE,CMDSend_Event,WAIT_REPLY,ec2x_queue_send},
	{WAIT_REPLY,ReplyScs_Event,AT_MODE,ec2x_reply_handler},
	{WAIT_REPLY,Timeout_Event,AT_MODE,ex2x_timeout_handler},
	{TRANS_MODE,MsgSend_Event,TRANS_MODE,NULL},
};

//定义事件队列
uint8_t event_buff[10];
Queue_HandleTypeDef ec2x_event_queue;
//定义tcp连接字符串
uint8_t tcp_connect[]="AT+QIOPEN=1,0,\"TCP\",\"115.236.153.170\",52894,0,2";

//定义tcp连接指令队列
uint8_t tcpconnct_step = 0;
ATMsg_TypeDef connectmsg_queue[] = {
   // {"AT\r\n","OK",500,5},
    {"AT+CPIN?\r\n","+CPIN: READY",500,5},
    {"AT+CREG?\r\n","+CREG: 0,1", 500, 2},
    {"AT+CGREG?\r\n","+CGREG: 0,1", 500, 2},
    {"AT+QICSGP=1,1,\"CMNET\",\"\",\"\",1\r\n","OK", 500, 3},
    {"AT+QIDEACT=1\r\n", "OK", 40, 1},
    {"AT+QIACT=1\r\n", "OK", 150, 1},
    {tcp_connect, "+QIOPEN:", 150, 5},
};

//定义发送消息队列
ATMsg_TypeDef ec2x_txmsg_buff[5];
Queue_HandleTypeDef ec2x_txmsg_queue;
ATMsg_TypeDef *cur_msg = NULL;

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
    // if(fd >= sizeof(ec2x_dev)/ sizeof(EC2x_HandleTypeDef))
    //     return (err_code = -1);
    // else{
    //     EC2x_HandleTypeDef *device = &ec2x_dev[fd];
    //     HAL_GPIO_WritePin(device->pwren_port, device->pwren_pin, GPIO_PIN_SET);
    //     HAL_Delay(500);
    //     HAL_GPIO_WritePin(device->pwren_port, device->pwren_pin, GPIO_PIN_RESET);
    // }

    ec2x_waitreply("RDY\r\n",10000);
    // ATMsg_TypeDef *msg_send = (ATMsg_TypeDef *)malloc(sizeof(ATMsg_TypeDef));
    // char *reply_data = (char *)malloc(sizeof("RDY\r\n"));

    // memcpy(reply_data,"RDY\r\n",sizeof("RDY\r\n"));
    
    // msg_send->expect_reply = reply_data;
    // msg_send->retry_times = 0;
    // msg_send->timeout = 10000;
    // msg_send->txmsg = NULL;

    // queue_insert(&ec2x_txmsg_queue,msg_send);
    queue_insert(&ec2x_event_queue,(uint8_t)Timeout_Event);
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
            queue_init(&ec2x_txmsg_queue,ec2x_txmsg_buff,5);
        
           // ec2x_fsm = (FSM_T*)malloc(sizeof(FSM_T));
            uint8_t num = sizeof(ec2x_transtable)/sizeof(ec2x_transtable[0]);
            FSM_Regist(&ec2x_fsm,ec2x_transtable,num,UINIT);
            queue_insert(&ec2x_event_queue,(uint8_t)Timeout_Event);
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

/**
 * @brief 回复匹配函数
 *        1）匹配成功，则插入回复成功事件
 *        2.1）匹配失败，超时次数-1
 *        2.2）匹配失败，超时次数=0，插入超时事件消息
 * 
 * @param expect 
 * @return int 
 */
int ec2x_replymatch(char* expect){
    uint8_t rx_buff[300]  = {0x00};
    int bytes_num = 0;
    bytes_num =  ec2x_read(0,rx_buff,200);
    if(bytes_num > 0)
    {
        if(strstr(rx_buff,expect) != NULL){
             shellPrint(&shell,"recv success");
             queue_insert(&ec2x_event_queue,(uint8_t)ReplyScs_Event);
            return 1;
        }
    }
    
    if(timeout_cnt == 0){
        shellPrint(&shell,"no valid data\r\n");
        queue_insert(&ec2x_event_queue,(uint8_t)Timeout_Event);
    }
    else
        timeout_cnt--;
    return 0;
}

//扫描发送消息队列是否有消息，有则发送队首消息
void ec2x_queue_send(void){
    
    if(!queue_is_empty(&ec2x_txmsg_queue)){
        if(queue_pull(&ec2x_txmsg_queue,&cur_msg,1))
            ec2x_cmd_send(0,cur_msg->txmsg,sizeof(cur_msg->txmsg),cur_msg->expect_reply,cur_msg->timeout);
    }      
}

/**
 * @brief 接收回复成功回调函数
 *        1）将消息pop出队列
 *        2）如果出队后消息队列还有元素，则插入指令发送事件
 * 
 */
void ec2x_reply_handler(void){

    if(!queue_is_empty(&ec2x_txmsg_queue)){
        queue_pop(&ec2x_txmsg_queue,&cur_msg,1);
        free(cur_msg->expect_reply);
        cur_msg->expect_reply = NULL;
        free(cur_msg->txmsg);
        cur_msg->txmsg = NULL;
        free(cur_msg);
        cur_msg = NULL;
    }
    if(!queue_is_empty(&ec2x_txmsg_queue)){
        queue_insert(&ec2x_event_queue,(uint8_t)CMDSend_Event);
    }        
}

/**
 * @brief 超时处理回调函数
 *        如果retry_times > 0 则减1，再插入指令发送事件
 *        如果retry_times = 0 则将消息出队
 * 
 */
void ex2x_timeout_handler(void){
    cur_msg->retry_times--;
    if(cur_msg->retry_times == 0){
        if(!queue_is_empty(&ec2x_txmsg_queue)){
            queue_pop(&ec2x_txmsg_queue,&cur_msg,1); 
            free(cur_msg->expect_reply);
            cur_msg->expect_reply = NULL;
            free(cur_msg->txmsg);
            cur_msg->txmsg = NULL;
            free(cur_msg);
            cur_msg = NULL; 
        }     
    }
    else
        queue_insert(&ec2x_event_queue,(uint8_t)CMDSend_Event); 
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
void ec2x_cmd_send(int fd,uint8_t *tx_buff,uint16_t len,uint8_t *expect_reply,uint16_t timeout)
{
    if(tx_buff !=NULL)
        ec2x_write(fd,tx_buff,len);
    ec2x_waitreply(expect_reply,timeout);
}

/**
 * @brief ec2x 进入等待回复状态
 *        拷贝期待回复指令
 *        插入timer列表
 * 
 * @param expect 
 * @param timeout 
 * @return * void 
 */
void ec2x_waitreply(uint8_t *expect,uint16_t timeout){
    timeout_cnt = timeout / 100;
    timer_insert(100,timeout_cnt,ec2x_replymatch,expect);
}

/**
 * @brief 状态机运行函数
 * 
 */
void ec2x_fms_proccess(void){
    if(queue_is_empty(&ec2x_event_queue) == 0){
        EC2x_Event_T evt = 0x00;
        queue_pop(&ec2x_event_queue,&evt,1);
        shellPrint(&shell,"Cur->State:%d ,Evt:%d,",(&ec2x_fsm)->state,evt);
        FSM_EventHandle(&ec2x_fsm,evt);
        shellPrint(&shell,"Next->State:%d\r\n",(&ec2x_fsm)->state);
    }
}
/**
 * @brief tcp 连接流程
 * 
 * @param fd 
 * @return uint8_t 
 */
uint8_t ec2x_tcp_connect(int fd){
    uint8_t err_code = 0;
    uint8_t i = 0;
    for(i = 0; i < sizeof(connectmsg_queue)/sizeof(ATMsg_TypeDef); ++i)
    {
        queue_insert(&ec2x_txmsg_queue,(int *)&connectmsg_queue[i]);
    }
    queue_insert(&ec2x_event_queue,(uint8_t)CMDSend_Event);
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
    uint8_t *tx_data,reply_data,tx_len = 0,reply_len = 0;
    tx_len = strlen(tx_buff);
    reply_len = strlen(expect_reply);
    if(strcmp(tx_buff,"tcp_connect")==0){
        ec2x_tcp_connect(fd);
    }
    else{
        ATMsg_TypeDef *msg_send = (ATMsg_TypeDef *)malloc(sizeof(ATMsg_TypeDef));
        tx_data = (char *)malloc((tx_len+2)*sizeof(uint8_t));
        reply_data = (char *)malloc((reply_len)*sizeof(uint8_t));

        memcpy(tx_data,tx_buff,tx_len);
        tx_data[tx_len] = 0x0D;
        tx_data[tx_len+1] = 0x0A;

        memcpy(reply_data,expect_reply,reply_len);
        
        msg_send->expect_reply = reply_data;
        msg_send->retry_times = timeout / 100;
        msg_send->timeout = timeout;
        msg_send->txmsg = tx_data;

        queue_insert(&ec2x_txmsg_queue,msg_send);
        queue_insert(&ec2x_event_queue,(uint8_t)CMDSend_Event);
        
    }
}

SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
ec2x_send, EC2x_Test, ec2x test);


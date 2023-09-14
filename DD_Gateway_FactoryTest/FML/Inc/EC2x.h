
#ifndef EC2X_H 
#define EC2X_H 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "main.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
typedef struct 
{
    uint8_t *txmsg;
    uint8_t *expect_reply;
    uint16_t timeout;
    uint8_t retry_times;
}ATMsg_TypeDef;

typedef struct 
{
    /* data */
    _Bool block;
    uint8_t *data;
    uint16_t head;
    uint16_t tail;
    uint16_t size;
    uint16_t capacity;
}RingBuffer_TypeDef;

typedef struct EC2x
{
    /* data */
    char dev_name[15];
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *pwren_port; // 复位控制引脚所在的GPIO
    uint16_t pwren_pin; // 复位控制引脚 
    GPIO_TypeDef *reset_port; // 复位控制引脚所在的GPIO
    uint16_t reset_pin; // 复位控制引脚 
    GPIO_TypeDef *wake_port; // 休眠控制引脚所在的GPIO
    uint16_t wake_pin; // 休眠控制引脚 wake
    RingBuffer_TypeDef msg;
}EC2x_HandleTypeDef;


enum  EC2x_States
{
    UINIT = 0x00, AT_MODE , WAIT_REPLY, TRANS_MODE
};

typedef enum{
    Init_Event = 0x00, Timeout_Event ,ReplyScs_Event, CMDSend_Event,
    MsgSend_Event,
}EC2x_Event_T;

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
int ec2x_open(char *dev_name);
void ec2x_handler(UART_HandleTypeDef *huart,uint8_t *data,uint16_t len);
void ec2x_fms_proccess(void);
uint8_t ec2x_tcp_connect(int fd);
static int ec2x_reset(int fd);
static int ec2x_cmd_send(int fd,uint8_t *tx_buff,uint16_t len,uint8_t *expect_reply,uint16_t timeout);
void EC2x_Test(int fd,char *tx_buff,char *expect_reply,uint16_t timeout);

/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	/* EC2X_H */

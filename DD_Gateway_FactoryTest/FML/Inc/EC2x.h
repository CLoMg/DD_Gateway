
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
    GPIO_TypeDef *reset_port; // 复位控制引脚所在的GPIO
    uint16_t reset_pin; // 复位控制引脚 
    GPIO_TypeDef *wake_port; // 休眠控制引脚所在的GPIO
    uint16_t wake_pin; // 休眠控制引脚 wake
    RingBuffer_TypeDef msg;
}EC2x_HandleTypeDef;

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
int atgm_read(int fd,uint8_t *buff,...);
int atgm_open(char *dev_name);

/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	/* EC2X_H */

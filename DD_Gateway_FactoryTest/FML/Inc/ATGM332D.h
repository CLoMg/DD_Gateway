
#ifndef ATGM332D_H 
#define ATGM332D_H 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "main.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

typedef struct ATGM332D
{
    /* data */
    char dev_name[15];
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *en_port; // DE控制引脚所在的GPIO
    uint16_t en_pin; // DE控制引脚
    uint8_t *data_buff;
    uint16_t data_index;
    uint16_t capacity;
}ATGM332D_HandleTypeDef;

/*----------------------------------variable----------------------------------*/
extern int bds_dev;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
int atgm_read(int fd,uint8_t *buff,uint16_t len);
int atgm_open(char *dev_name);
void BDS_Test(char *cmd,...);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	/* ATGM332D_H */

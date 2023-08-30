/**
 * @file queue.h
 * @brief 
 * @author CL
 * @version 1.0
 * @date 2023-08-30
 * 
 * @copyright Copyright (c) 2023  JiuTong
 * 
 */
#ifndef QUEUE_H 
#define QUEUE_H 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "main.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
typedef struct queue
{
    /* data */
    uint8_t block;
    uint16_t head;
    uint16_t tail;
    int *buff;
    uint16_t capacity;
    uint16_t size;
}Queue_HandleTypeDef;

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	/* QUEUE_H */

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
    uint8_t *buff;
    uint16_t capacity;
}Queue_HandleTypeDef;

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief 初始化队列
 */
void queue_init(Queue_HandleTypeDef *self,void *buff,uint16_t capacity);
/**
 * @brief 插入元素到队列
 */
int queue_insert(Queue_HandleTypeDef *self,uint8_t item);
/**
 * @brief 拉取队列len个元素（不从队列中删除）
 */
int queue_pull(Queue_HandleTypeDef *self,int *temp,uint16_t len);

/**
 * @brief  弹出队列len个元素（要从队列中删除）
 */
int queue_pop(Queue_HandleTypeDef *self,int *temp,uint16_t len);
/**
 * @brief 判断队列是否满
 */
int queue_is_full(Queue_HandleTypeDef *self);

/**
 * @brief 判断队列是否为空
 */
int queue_is_empty(Queue_HandleTypeDef *self);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	/* QUEUE_H */

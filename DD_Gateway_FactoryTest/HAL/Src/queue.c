/**
 * @file queue.c
 * @brief 
 * @author CL
 * @version 1.0
 * @date 2023-08-30
 * 
 * @copyright Copyright (c) 2023  JiuTong
 * 
 */
#include "queue.h"

/**
 * @brief 
 * @param  self             My Param doc
 * @param  buff             My Param doc
 * @param  size             My Param doc
 */
void queue_init(Queue_HandleTypeDef *self,void *buff,uint16_t capacity,uint16_t size){
    self->block = 0;
    self->head = 0;
    self->tail = 0;
    self->capacity = capacity; 
    self->buff = buff;
    self->size = 0;
}
/**
 * @brief 
 * @param  self             My Param doc
 * @param  item             My Param doc
 * @return int 
 */
int queue_insert(Queue_HandleTypeDef *self,int *item){
    if(self->block  == 0){
        self->block = 1;
        self->buff[self->tail] = &item;
        self->tail = (self->tail +1) % self->capacity;
        self->size++;
        self->size = self->size > self->capacity ? self->capacity : self->size;
        if((self->tail > self->head)&&(self->size == self->capacity))
            self->head = self->tail;
        self->block = 0;
        return 1;
    }

    return -1;
}
/**
 * @brief 拉取队列len个元素（不从队列中删除）
 * @param  self             My Param doc
 * @param  temp             My Param doc
 * @param  len              My Param doc
 * @return int 
 */
int queue_pull(Queue_HandleTypeDef *self,int *temp,uint16_t len){
    uint16_t j = 0 ;
    //如果队列被上锁了，则等待100ms
    if(self->block == 1)
    {
        HAL_Delay(100);
        if(self->block == 1)
            return -1;
    }
    else{
        self->block = 1;
        if(len > self->size)
            len = self->size;
        for(j = 0 ; j < len; j++){
            temp[j] = self->buff[(self->head+j)% self->capacity];
        }
        self->block = 0;
    }
    return len;
}

/**
 * @brief  弹出队列len个元素（要从队列中删除）
 * @param  self             My Param doc
 * @param  temp             My Param doc
 * @param  len              My Param doc
 * @return int 
 *          -1: 队列无法访问
 *          len： 弹出元素数量
 */
int queue_pop(Queue_HandleTypeDef *self,int *temp,uint16_t len){
    uint16_t j = 0 ;
    //如果队列被上锁了，则等待100ms
    if(self->block == 1)
    {
        HAL_Delay(100);
        if(self->block == 1)
            return -1;
    }
    else{
        self->block = 1;
        if(len > self->size)
            len = self->size;
        for(j = 0 ; j < len; j++){
            temp[j] = self->buff[self->head];
            self->head = (self->head + 1) % self->capacity;
            self->size--;
        }
        self->block = 0;
    }
    return len;
}

/**
 * @brief 判断队列是否满
 * @param  self             My Param doc
 * @return int 
 *  *       -1: 队列无法访问
 *          1: 队列满
 *          0: 队列未满
 */

int queue_is_empty(Queue_HandleTypeDef *self){
    int re_code = -1;
    if(self->block == 1)
    {
        HAL_Delay(100);
        if(self->block == 1)
            re_code = -1;
    }
    else{
        self->block = 1;
       if(self->size == self->capacity)
        re_code = 1;
        else
        re_code = 0;
        self->block = 1;
    }
    return re_code;
}

/**
 * @brief 判断队列是否为空
 * @param  self             My Param doc
 * @return int 
 *         -1: 队列无法访问
 *          1: 队列为空
 *          0: 队列非空
 */

int queue_is_full(Queue_HandleTypeDef *self){
    int re_code = -1;
    if(self->block == 1)
    {
        HAL_Delay(100);
        if(self->block == 1)
            re_code = -1;
    }
    else{
        self->block = 1;
       if(self->size == 0)
        re_code = 1;
        else
        re_code = 0;
        self->block = 1;
    }
    return re_code;
}
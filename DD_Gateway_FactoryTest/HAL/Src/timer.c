/**
 ******************************************************************************
*
* @file   
*
* @brief   
*
* @attention
*
*Copyright (c) 2023 JiuTong.
*
*All rights reserved.
 ******************************************************************************
*/

/*----------------------------------include-----------------------------------*/
#include "timer.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
uint32_t base_tick = 0; //定义定时基准tick

Timer_HandleTypeDef *timer_list = NULL;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief 定时列表初始化函数
 * 
 */
void timer_init(void){
    timer_list = (Timer_HandleTypeDef *)malloc(sizeof(Timer_HandleTypeDef));
    timer_list->next = NULL;
}
/**
 * @brief 定时器更新函数
 *        需要将此函数作为定时触发中断的回调函数，定时更新base_tick
 *        每次更新后比较链表中是否有定时到达，触发对应回调函数
 * 
 */
void timer_update(void){
    base_tick++;
    Timer_HandleTypeDef *p  = timer_list, *q = NULL;
    while(p->next !=NULL){
        uint32_t err_tick;
        err_tick = base_tick > p->next->init_tick ?  (base_tick - p->next->init_tick) :  (0xFFFFFFFF - p->next->init_tick + base_tick);
        if(err_tick >= p->next->timeout){
            //执行回调函数，回调函数要求时间尽可能短
            p->next->func(p->next->param);
            if(p->next->reenter_times == 0){
                q = p->next;  
                p->next = q->next;
                free(q);
                q = NULL;
            }
            else if(p->next->reenter_times == -1){
                p->next->init_tick = base_tick;
                p = p->next;
            }
            else{
                p->next->reenter_times--;
                p->next->init_tick = base_tick;
                p = p->next;
            }
        }
        else
            p = p->next;
    }
}

/**
 * @brief 定时事件列表插入函数（尾插法）
 * 
 * @param timeout 定时时间
 * @param timeout 重入次数，-1 为无限循环，0为不可重入，正整数为重入次数
 * @param func 回调函数
 */
void timer_insert(uint32_t timeout,int8_t cycles,void *func,void *param){
    Timer_HandleTypeDef *new,*p;
    new = (Timer_HandleTypeDef *)malloc(sizeof(Timer_HandleTypeDef));
    p = timer_list;

    new->func = func;
    new->param = param;
    new->next = NULL;
    new->init_tick = base_tick;
    new->timeout = timeout;
    new->reenter_times = cycles;

    while(p->next != NULL)
        p = p->next;
    p->next  = new;    
};
/*------------------------------------test------------------------------------*/

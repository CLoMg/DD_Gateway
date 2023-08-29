
#ifndef TIMER_H 
#define TIMER_H 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "main.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
typedef struct timer
{
    /* data */
    uint32_t  init_tick;
    uint32_t timeout;
    int8_t reenter_times;
    void  (*func)(void *);
    void *param;
    struct  timer *next;    
}Timer_HandleTypeDef;

/*----------------------------------variable----------------------------------*/
extern uint32_t base_tick;
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief 定时列表初始化函数
 * 
 */
void timer_init(void);

/**
 * @brief 定时器更新函数
 *        需要将此函数作为定时触发中断的回调函数，定时更新base_tick
 *        每次更新后比较链表中是否有定时到达，触发对应回调函数
 * 
 */
void timer_update(void);

/**
 * @brief 定时事件列表插入函数（尾插法）
 * 
 * @param timeout 定时时间
 * @param timeout 重入次数，-1 为无限循环，0为不可重入，正整数为重入次数
 * @param func 回调函数
 */
void timer_insert(uint32_t timeout,int8_t cycles,void *func,void *param);
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	/* TIMER_H */

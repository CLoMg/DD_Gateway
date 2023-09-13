
#ifndef LED_H 
#define LED_H 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "main.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/
// 定义LED结构体，包含GPIO端口和引脚信息
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} LED_HandleTypeDef;
/*----------------------------------variable----------------------------------*/
extern LED_HandleTypeDef dev_led[];
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/

// 初始化LED
void LED_Init(LED_HandleTypeDef *led);

// 打开LED
void LED_On(LED_HandleTypeDef *led);

// 关闭LED
void LED_Off(LED_HandleTypeDef *led);

// 翻转LED状态
void LED_Toggle(LED_HandleTypeDef *led);
void LED1_Toggle(void);
// 闪烁LED
void LED_Blink(LED_HandleTypeDef *led,uint32_t gap,uint8_t cycle);
/*------------------------------------test------------------------------------*/
void LED_Test(char *name, char *type, ...);

#ifdef __cplusplus
}
#endif

#endif	/* LED_H */

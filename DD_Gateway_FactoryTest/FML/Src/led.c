
/*----------------------------------include-----------------------------------*/
#include "led.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
LED_HandleTypeDef dev_led[] = {
    {
        GPIOB,
        GPIO_PIN_2
    },
    {
        GPIOC,
        GPIO_PIN_13
    }
};
/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief 
 * 
 * @param led 
 */
void LED_Init(LED_HandleTypeDef *led) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIO时钟
    __HAL_RCC_GPIOB_CLK_ENABLE(); // 替换为实际的GPIO时钟
    __HAL_RCC_GPIOC_CLK_ENABLE(); // 替换为实际的GPIO时钟
    // 配置GPIO引脚
    GPIO_InitStruct.Pin = led->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(led->port, &GPIO_InitStruct);
    
    // 关闭LED初始状态
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

/**
 * @brief 
 * 
 * @param led 
 */
void LED_On(LED_HandleTypeDef *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}
/**
 * @brief 
 * 
 * @param led 
 */
void LED_Off(LED_HandleTypeDef *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}
/**
 * @brief 
 * 
 * @param led 
 */
void LED_Toggle(LED_HandleTypeDef *led) {
    HAL_GPIO_TogglePin(led->port, led->pin);
}

void LED1_Toggle(void){
    LED_Toggle(&dev_led[0]);
}
/**
 * @brief 
 * 
 * @param led 
 * @param gap 
 * @param cycle 
 */
void LED_Blink(LED_HandleTypeDef *led,uint32_t gap,uint8_t cycle){
    uint8_t i = 0;
    for(i = 0; i < cycle * 2; i++){
        HAL_GPIO_TogglePin(led->port,led->pin);
        HAL_Delay(gap);
    } 
}
/*------------------------------------test------------------------------------*/

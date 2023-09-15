
/*----------------------------------include-----------------------------------*/
#include "led.h"
#include "shell_port.h"
#include <stdarg.h>
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
        Delay_ms(gap);
    } 
}
/*------------------------------------test------------------------------------*/
/**
 * @brief LED测试函数
 * 
 * @param name LED接口（D6/D7）
 * @param type LED状态 ON/OFF/TOGGLE/BLINK
 * @param ...  可变参数（仅type为blink时有效）args[0]:gap(闪烁间隔ms) args[1]:cycle(循环次数)
 */
void LED_Test(char *name, char *type, ...){
  LED_HandleTypeDef *test_led;
  uint32_t gap = 0;
  uint8_t cycle = 0;

  if(strcmp(name,"D6") == 0)
    test_led = &dev_led[0];
  else if((strcmp(name,"D7") == 0))
    test_led = &dev_led[1];
  else{
    shellPrint(&shell,"invalid led name");
    return;
  }

  if(strcmp(type,"on") == 0)
    LED_On(test_led);
  else if(strcmp(type,"off") == 0)
    LED_Off(test_led);
  else if(strcmp(type,"toggle") == 0)
    LED_Toggle(test_led);
  else if(strcmp(type,"blink") == 0)
  {
    va_list p_args;
    va_start(p_args,type);
    gap = va_arg(p_args,int);
    cycle = va_arg(p_args,int);
    va_end(p_args);
    LED_Blink(test_led, gap, cycle);
  }
  else{
    shellPrint(&shell,"invalid cmd type");
    return;
  }
}
/**
 * @brief 
 * 
 */
SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
led_test, LED_Test, test led func);
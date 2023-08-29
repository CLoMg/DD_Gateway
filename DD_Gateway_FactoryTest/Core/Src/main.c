/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "led.h"
#include "adc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "timer.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "shell_port.h"
#include "stdint.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "rs485.h"
#include "W25QXX.h"
#include "ATGM332D.h"
#include "EC2x.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int bds_dev = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  timer_init();//定时事件列表初始化，需要在时基更新函数运行之前进行
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  LED_Init(&dev_led[0]);
  LED_Init(&dev_led[1]);
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* TODO:fix shell initialize */
  User_Shell_Init();
  BSP_W25Qx_Init();
  bds_dev = atgm_open("uart2/bds_0");
  ec2x_open("uart1/ec200s");
  timer_insert(1000,-1,LED_Toggle,&dev_led[0]);
  timer_insert(1000,10,LED_Toggle,&dev_led[1]);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
  while (1)
  {
   
    /* USER CODE END WHILE */
    /* ADC 电压监测接口测试*/
    /* LED 测试 */ 
    /* 亮度监测接口测试 */
    
    /* RS485接口测试 */
     /* Flash 接口测试 */
     /* LORA 1发送 LORA2 接收测试*/
     /* LORA2发送 LORA1 接收测试*/
     /* Cat-1 测试 */
     /* ETH 测试 */
     /* ATGM223D接口测试 */
     
     HAL_Delay(100);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief 
 * 
 * @param a 
 * @param b 
 * @param str 
 * @param val 
 */
void LDR_Value_Get(int a, char b, char *str,float val)
{   
    uint32_t ldr_adc_value = 0;
    ldr_adc_value = ADC_Average_Get(10,50);
    shellPrint(&shell,"LDR:%d, input int:%d, input char:%c, input str:%s, input float:%.4f \r\n",ldr_adc_value,a,b,str,val);
}

/**
 * @brief 
 * 
 */
SHELL_EXPORT_CMD_AGENCY(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
LDR_Get, LDR_Value_Get, get ldr value,p1,(char)p2,(char *)p3,SHELL_PARAM_FLOAT(p4));

/**
 * @brief 
 * 
 * @param name 
 * @param type 
 * @param ... 
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
led, LED_Test, test led func);

/**
 * @brief 
 * 
 */
void Battery_Test(void)
{   
    float voltage = 0;
    voltage = (float)ADC_Average_Get(5,50)/4096 * 3.3 * 2;
    
    shellPrint(&shell,"voltage:%.2fv\r\n",voltage);
}

/**
 * @brief 
 * 
 */
SHELL_EXPORT_CMD_AGENCY(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
battery, Battery_Test, get battery voltage value);

/**
 * @brief 
 * 
 * @param str 
 */
void RS485_Test(char *str)
{
  RS485_SendData(&dev_rs485[0], (uint8_t *)str, strlen(str));
}

SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
rs485_send, RS485_Test, rs485 send test);

/**
 * @brief 
 * 
 * @param str 
 */
void Flash_Test(void)
{
  uint8_t flash_id[2] = {0x00,};
  uint8_t JEDEC_id[3] = {0x00,};
  uint32_t cap_kb = 0;
  float cap_mb = 0.0f;

  BSP_W25Qx_Read_ID(flash_id);
  BSP_W25Qx_Read_JEDEC_ID(JEDEC_id);
  cap_kb = pow(2 , (JEDEC_id[2] - 10)) *8 ;
  cap_mb = (float)cap_kb / 1024.0;

  shellPrint(&shell,"Manu_ID:0x%02x,Dev_ID:0x%02x\r\nManu_Identi:0x%02x,Memory_Type:0x%02x\r\n\
  Capacity:0x%02x = %dK-Bit = %.2fM-Bit\r\n",flash_id[0],flash_id[1],JEDEC_id[0],JEDEC_id[1],JEDEC_id[2],cap_kb,cap_mb);
}

SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
flash_test, Flash_Test, rs485 send test);

void BDS_Test(char *cmd,...)
{
  LED_HandleTypeDef *test_led;
  int param_1 = -1;
  uint16_t len;
  uint8_t buff[300];

  if(strcmp(cmd,"read") == 0)
  {
    va_list p_args;
    va_start(p_args,cmd);
    param_1 = va_arg(p_args,int);
    va_end(p_args);
    len  = atgm_read(bds_dev,buff,param_1);

    if(len > 0)
      shellPrint(&shell,buff,len);
    memset(buff,0,300);
  }
}
SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
bds_test, BDS_Test, bde_dev test);

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


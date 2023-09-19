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
#include "math.h"

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
#include "SX1278.h"
// #include "y_lora.h"


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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DeviceID_Print(void);
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


  //Lora_Init();
  SX1278_init(&Lora_dev[0], Lora_dev[0].frequency, Lora_dev[0].power,
		Lora_dev[0].LoRa_SF, Lora_dev[0].LoRa_BW, Lora_dev[0].LoRa_CR,
		Lora_dev[0].LoRa_CRC_sum, Lora_dev[0].packetLength);
   SX1278_receive(&Lora_dev[0], 100, 500);
  SX1278_init(&Lora_dev[1], Lora_dev[1].frequency, Lora_dev[0].power,
		Lora_dev[0].LoRa_SF, Lora_dev[0].LoRa_BW, Lora_dev[0].LoRa_CR,
		Lora_dev[0].LoRa_CRC_sum, Lora_dev[0].packetLength);
   SX1278_receive(&Lora_dev[1], 100, 500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 
  while (1)
  {
    // uint16_t shellidle_upper = 30;
    /* USER CODE END WHILE */
    /*电量LED D6 D7*/
    LED_On(&dev_led[0]);
    LED_On(&dev_led[1]);
    /*Mac地址打印*/
    DeviceID_Print();
    /* ADC 电压监测接口测试*/
    Battery_Test();
    /* LED 测试 */ 
    // LED_Blink(&dev_led[0],500,10); //LED0每隔500ms翻转一次
    // LED_Blink(&dev_led[1],1000,10);//LED1每个1000ms翻转一次
    /* 亮度监测接口测试 */
    LDR_Value_Get();
    /* RS485接口测试 */
    RS485_Test("RS485 TRANSMIT OK\r\n");
     /* Flash 接口测试 */
    Flash_Test();
    /* ATGM223D接口测试 */
    BDS_Test("read",100);
     /* LORA 1发送 LORA2 接收测试*/
    LORA_Send(0,"Lora0 tx test\r\n",200);
    Delay_ms(500);
     /* LORA2发送 LORA1 接收测试*/
    LORA_Send(1,"Lora1 tx test\r\n",200);
    Delay_ms(500);
     /* Cat-1 测试 */
    EC2x_Test(0,"AT","OK",500);
    shellPrint(&shell,"Test OK Count:%d/8\r\n",test_ok_cnt);
    shellPrint(&shell,"Type [help] for the list of instructions\r\n");
    while(1){
      // shellidle_cnt++;
      // if((shellidle_cnt > 10*10)&&(shellidle_cnt % 10 == 0)){
      //   shellPrint(&shell,"If no instruction is entered within %ds, it will jump to the app program\r\n", shellidle_upper - shellidle_cnt/10);
      // }
      // if(shellidle_cnt > shellidle_upper*10)
      // {
      //    shellPrint(&shell,"It will jump to the app program now\r\n");
      //    shellidle_cnt = 0;
      // }
      Delay_ms(100);
    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
/* USER CODE BEGIN 4 */
/**
 * @brief 
 * 
 * @param fd 
 * @param tx_buff 
 * @param expect_reply 
 * @param timeout 
 */
void Test_All(void)
{
    test_ok_cnt = 0;
        /* USER CODE END WHILE */
    /* ADC 电压监测接口测试*/
    Battery_Test();
    /* LED 测试 */ 
    // LED_Blink(&dev_led[0],500,10); //LED0每隔500ms翻转一次
    // LED_Blink(&dev_led[1],1000,10);//LED1每个1000ms翻转一次
    /* 亮度监测接口测试 */
    LDR_Value_Get();
    /* RS485接口测试 */
    RS485_Test("RS485 TRANSMIT OK\r\n");
     /* Flash 接口测试 */
    Flash_Test();
    /* ATGM223D接口测试 */
    BDS_Test("read",100);
     /* LORA 1发送 LORA2 接收测试*/
    LORA_Send(0,"Lora0 tx test\r\n",200);
     /* LORA2发送 LORA1 接收测试*/
    LORA_Send(1,"Lora1 tx test\r\n",200);

     /* Cat-1 测试 */
    EC2x_Test(0,"AT","OK",100);
    shellPrint(&shell,"Test OK Count:%d/8\r\n",test_ok_cnt);
}

SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
test_all, Test_All, test all device);

/**
 * @brief printf Device ID 8bytes
 * 
 */
void DeviceID_Print(void){
    int i = 0;
    shellPrint(&shell,"MAC:");

    for(i = 0;i < 2 ;++i){
      uint32_t u32_mac = 0xffff;
      u32_mac = *((uint32_t *)(0x1FFF7594 - i * 4));
      //shellPrint(&shell,"%x",u32_mac);
      shellPrint(&shell,"%02x",(uint8_t)(u32_mac >> 24));
      shellPrint(&shell,"%02x",(uint8_t)(u32_mac >> 16));
      shellPrint(&shell,"%02x",(uint8_t)(u32_mac >> 8));
      shellPrint(&shell,"%02x",(uint8_t)(u32_mac ));
    }
    shellPrint(&shell,"\r\n");
}

SHELL_EXPORT_CMD(
SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|SHELL_CMD_DISABLE_RETURN,
mac_print, DeviceID_Print, printf mac_id);
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


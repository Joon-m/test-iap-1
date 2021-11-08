/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"stm32f1xx_hal_flash.h"
#include "stdio.h"
#include "stddef.h"
#include "usbd_cdc_if.h"
#include"stm32f1xx_hal_rtc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FLASH_APP_ADDR 0x800B000
typedef void (*pFunction)(void);


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
void FLASH_EEPROM_Write(uint32_t n)
{
    HAL_FLASH_Unlock();     //解锁
    uint32_t PageError = 0;
    static FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress=0x0800F500;
    EraseInitStruct.NbPages = 1;
    if (HAL_FLASHEx_Erase(&EraseInitStruct,&PageError) == HAL_OK) //如果结构体中的起始地�??????????0x0801FC00，这�??????????页的数据擦除成功，返回OK
    {
//        printf("擦除 成功\r\n");
    }
    uint32_t writeFlashData = n;        //代写入的�??????????
    uint32_t addr = 0x0800F500;                  //写入的地�??????????
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,addr, writeFlashData); //向FLASH中写�??????????
//    printf("at address:0x%x, read value:%d\r\n", addr, *(__IO uint32_t*)addr);
    HAL_FLASH_Lock();
}

void FLASH_EEPROM_Write_BYTES(uint32_t n,uint32_t address)
{
    HAL_FLASH_Unlock();     //解锁
    uint32_t PageError = 0;
    static FLASH_EraseInitTypeDef EraseInitStruct;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress=address;
    EraseInitStruct.NbPages = 1;
    if(address%1024==0)
    {
    if (HAL_FLASHEx_Erase(&EraseInitStruct,&PageError) == HAL_OK) //如果结构体中的起始地�??????????0x0801FC00，这�??????????页的数据擦除成功，返回OK
    {
//        printf("擦除 成功\r\n");
    }
    }
    uint32_t writeFlashData = n;        //代写入的�??????????
    uint32_t addr = address;                  //写入的地�??????????
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,addr, writeFlashData); //向FLASH中写�??????????
//    printf("at address:0x%x, read value:%d\r\n", addr, *(__IO uint32_t*)addr);
    HAL_FLASH_Lock();
}

uint32_t FLASH_EEPROM_Read(void)
{
    HAL_FLASH_Unlock();
    uint32_t Page = 0;
    uint32_t addr = 0x0800F400;                  //写入的地�??????????
    Page=*(__IO uint32_t*)addr;
    return Page;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef void (*iapfun)(void);
iapfun jump2app;
uint16_t iapbuf[1024];
//跳转到应用程序段
//app
void MSR_MSP(uint32_t addr)
{
	__ASM volatile(" MSR MSP, r0");
	__ASM volatile(" BX r14");
}



void iap_load_app(uint32_t appxaddr)
{
	if(((*(vu32*)(appxaddr+4))&0xFF000000)==0x08000000)//判断是否�???????????????0X08XXXXXX.
	{
		jump2app=(iapfun)*(vu32*)(appxaddr+4);		//用户代码区第二个字为程序�???????????????始地�???????????????(复位地址)
//		MSR_MSP(*(vu32*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)

		jump2app();									//跳转到APP.
	}
}

int fputc(int ch,FILE *f)
{
	size_t count =1;
	char chTemp=(char)ch;
	HAL_UART_Transmit(&huart1,(uint8_t*)&chTemp,count,100);

	return count;
}

uint8_t aRxBuffer[4];
uint32_t aRxnumbuf[2000];
uint32_t aRxnum=0;
uint8_t rDataFlag = 0;
uint8_t updateFlag = 0;
extern USBD_HandleTypeDef hUsbDeviceFS;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  aRxnumbuf[aRxnum]=aRxBuffer[3];
  aRxnumbuf[aRxnum]=(aRxnumbuf[aRxnum]<<8)+aRxBuffer[2];
  aRxnumbuf[aRxnum]=(aRxnumbuf[aRxnum]<<8)+aRxBuffer[1];
  aRxnumbuf[aRxnum]=(aRxnumbuf[aRxnum]<<8)+aRxBuffer[0];
//  if(aRxBuffer[0]!=0x00){
	  aRxnum++;
    if(aRxBuffer[0]==0x7E){  // (ascii)0x7E = "~"
      rDataFlag = 1;
    }
 // }
  while(HAL_UART_Receive_IT(&huart1, aRxBuffer, 4) != HAL_OK); // Wait completly receive 1 byte data, and put data in rDataBuffer
}
/*
 * 设置SP指针函数
 */
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef DateToUpdate = {0};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uint8_t aTxBuffer[]="flash update";
    uint32_t preTicks;


    uint32_t flash_address=0x0800B000;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
//  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
//  HAL_Delay(5000);
  HAL_UART_Receive_IT(&huart1,(uint8_t*)aRxBuffer,4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

// 	  if(KEY_PIN==0)
// 	  {
 		  if(rDataFlag==1)
 		  {
 			  HAL_UART_Transmit(&huart1,aRxnumbuf,aRxnum,0xffff);
 			  for(int i=0;i<aRxnum;i++)
 			  {
 		      FLASH_EEPROM_Write_BYTES(aRxnumbuf[i],flash_address);
 		     flash_address=flash_address+4;
 			  }
 			 aRxnum=0;
 			 rDataFlag=0;
 			updateFlag=1;
 		  }
 	 	  if(updateFlag==1)
 	 	  {
 	 		HAL_RCC_DeInit();
// 	 		HAL_UART_MspDeInit(&huart1);
 	 		USBD_DeInit(&hUsbDeviceFS);
 	 		updateFlag=0;
 	 		iap_load_app(FLASH_APP_ADDR);
 	 	  }
 	 	 //
 //     }

 //	  else
 //	  {
 //		 iap_load_app(FLASH_APP_ADDR);
 //	  }
 	 //
 //     }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_RESET);
  HAL_Delay(65);
  //先把PA12拉低再拉高，利用D+模拟USB的拔插动作
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,GPIO_PIN_SET);
  HAL_Delay(65);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

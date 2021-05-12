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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan3;

/* USER CODE BEGIN PV */
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t pTxMailbox;
uint8_t a[]={0, 0, 0, 0, 0, 0, 0, 0};
uint8_t r[8];
CAN_FilterTypeDef sFilterConfig;

CAN_HandleTypeDef hcan3;
CAN_TxHeaderTypeDef pTx3Header;
CAN_RxHeaderTypeDef pRx3Header;
uint32_t pTx3Mailbox;
uint8_t a3[]={0, 0, 0, 0, 0, 0, 0, 0};
uint8_t r3[8];
CAN_FilterTypeDef s3FilterConfig;

CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef pTx2Header;
CAN_RxHeaderTypeDef pRx2Header;
uint32_t pTx2Mailbox;
uint8_t a2[]={0, 0, 0, 0, 0, 0, 0, 0};
uint8_t r2[8];
CAN_FilterTypeDef s2FilterConfig;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN3_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN3_Init();
  /* USER CODE BEGIN 2 */
pTxHeader.DLC = 8;
pTxHeader.IDE = CAN_ID_STD;
pTxHeader.RTR = CAN_RTR_DATA;
pTxHeader.StdId = 0x244;
  
sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
sFilterConfig.FilterIdHigh = 0;
sFilterConfig.FilterIdLow = 0;
sFilterConfig.FilterMaskIdHigh = 0;
sFilterConfig.FilterIdLow = 0;
sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
sFilterConfig.FilterActivation = ENABLE;

HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);


HAL_CAN_Start(&hcan1);
HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
 

pTx3Header.DLC = 8;
pTx3Header.IDE = CAN_ID_STD;
pTx3Header.RTR = CAN_RTR_DATA;
pTx3Header.StdId = 0x247;
  
s3FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
s3FilterConfig.FilterIdHigh = 0;
s3FilterConfig.FilterIdLow = 0;
s3FilterConfig.FilterMaskIdHigh = 0;
s3FilterConfig.FilterIdLow = 0;
s3FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
s3FilterConfig.FilterActivation = ENABLE;

HAL_CAN_ConfigFilter(&hcan3, &s3FilterConfig);

HAL_CAN_Start(&hcan3);
HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO0_MSG_PENDING);
/*
pTx2Header.DLC = 8;
pTx2Header.IDE = CAN_ID_STD;
pTx2Header.RTR = CAN_RTR_DATA;
pTx2Header.StdId = 0x237;
  
s2FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
s2FilterConfig.FilterIdHigh = 0;
s2FilterConfig.FilterIdLow = 0;
s2FilterConfig.FilterMaskIdHigh = 0;
s2FilterConfig.FilterIdLow = 0;
s2FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
s2FilterConfig.FilterActivation = ENABLE;

HAL_CAN_ConfigFilter(&hcan2, &s2FilterConfig);

HAL_CAN_Start(&hcan2);
HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    for(int i=0;i<8;i++){a[i]++; a3[i]++; a2[i]++;}
  HAL_Delay(1000);
    HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, a, &pTxMailbox);
    HAL_CAN_AddTxMessage(&hcan3, &pTx3Header, a3, &pTx3Mailbox);
   // HAL_CAN_AddTxMessage(&hcan2, &pTx2Header, a2, &pTx2Mailbox);
    /* USER CODE END WHILE */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN3_Init(void)
{

  /* USER CODE BEGIN CAN3_Init 0 */

  /* USER CODE END CAN3_Init 0 */

  /* USER CODE BEGIN CAN3_Init 1 */

  /* USER CODE END CAN3_Init 1 */
  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 2;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN3_Init 2 */

  /* USER CODE END CAN3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

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
CAN_TxHeaderTypeDef pTxHeaderSergiu;
CAN_TxHeaderTypeDef pTxHeaderAndrei;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t pTxMailbox;
uint8_t a[]={0, 0, 0, 0, 0, 0, 0, 0};
uint8_t r[8];
uint8_t messageFromSergiu[8];
uint8_t checkKeyFromSergiu;
uint8_t messageFromAndrei[8];
CAN_FilterTypeDef sFilterConfig;

CAN_HandleTypeDef hcan3;
CAN_TxHeaderTypeDef pTx3Header;
CAN_TxHeaderTypeDef pTxHeaderBolaj;
CAN_TxHeaderTypeDef pTxHeaderNico;
CAN_TxHeaderTypeDef pTxHeaderIulia;
CAN_RxHeaderTypeDef pRx3Header;
uint32_t pTx3Mailbox;
uint8_t a3[]={0, 0, 0, 0, 0, 0, 0, 0};
uint8_t r3[8];
uint8_t messageFromBolaj[8];
uint8_t messageFromNico[8];
uint8_t messageFromIulia[8];
CAN_FilterTypeDef s3FilterConfig;
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
pTxHeader.StdId = 0x18;
 
pTxHeaderSergiu.DLC = 8;
pTxHeaderSergiu.IDE = CAN_ID_STD;
pTxHeaderSergiu.RTR = CAN_RTR_DATA;
pTxHeaderSergiu.StdId = 0x60;

pTxHeaderAndrei.DLC = 8;
pTxHeaderAndrei.IDE = CAN_ID_STD;
pTxHeaderAndrei.RTR = CAN_RTR_DATA;
pTxHeaderAndrei.StdId = 0x73;

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
pTx3Header.StdId = 0x19;
 
pTxHeaderBolaj.DLC = 8;
pTxHeaderBolaj.IDE = CAN_ID_STD;
pTxHeaderBolaj.RTR = CAN_RTR_DATA;
pTxHeaderBolaj.StdId = 0x11;

pTxHeaderNico.DLC = 8;
pTxHeaderNico.IDE = CAN_ID_STD;
pTxHeaderNico.RTR = CAN_RTR_DATA;
pTxHeaderNico.StdId = 0x21;

pTxHeaderIulia.DLC = 8;
pTxHeaderIulia.IDE = CAN_ID_STD;
pTxHeaderIulia.RTR = CAN_RTR_DATA;
pTxHeaderIulia.StdId = 0x13;

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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1000);
    if(checkKeyFromSergiu == 8){
       HAL_CAN_AddTxMessage(&hcan1, &pTxHeaderBolaj, messageFromBolaj, &pTxMailbox);
       HAL_CAN_AddTxMessage(&hcan3, &pTxHeaderAndrei, messageFromAndrei, &pTx3Mailbox);
       HAL_CAN_AddTxMessage(&hcan1, &pTxHeaderSergiu, messageFromSergiu, &pTxMailbox);
    }
    //for(int i=0;i<8;i++){a[i]++; a3[i]++;}
  
   // HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, a, &pTxMailbox);
   // HAL_CAN_AddTxMessage(&hcan3, &pTx3Header, a3, &pTx3Mailbox);
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

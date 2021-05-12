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
CAN_FilterConfTypeDef sFConfig1;
CanTxMsgTypeDef txmsg1;
CanRxMsgTypeDef rxmsg1;

char string[1];




extern uint32_t uwTick;
uint32_t old_uwTick;
CAN_FilterConfTypeDef sFConfig3;
CanTxMsgTypeDef txmsg3;
CanRxMsgTypeDef rxmsg3;

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
        
    txmsg1.StdId = 0x100;
    txmsg1.ExtId = 0x01;
    txmsg1.IDE = CAN_ID_STD;
    txmsg1.RTR = CAN_RTR_DATA;
    txmsg1.DLC = 8;
    txmsg1.Data[0]=2;
    txmsg1.Data[1]=2;
    txmsg1.Data[2]=2;
    txmsg1.Data[3]=2;
    txmsg1.Data[4]=2;
    txmsg1.Data[5]=2;
   txmsg1.Data[6]=2;
    txmsg1.Data[7]=2;
    hcan1.pTxMsg = &txmsg1; 
    hcan1.pRxMsg = &rxmsg1; 
    
     sFConfig1.FilterNumber = 0; // [0..13]
     sFConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
     sFConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
     sFConfig1.FilterIdHigh = 0; //COMP
     sFConfig1.FilterIdLow = 0;
     sFConfig1.FilterMaskIdHigh = 0; //MASK
     sFConfig1.FilterMaskIdLow = 0;
     sFConfig1.FilterFIFOAssignment = 0;
     sFConfig1.FilterActivation = ENABLE;
     sFConfig1.BankNumber = 14;
     HAL_CAN_ConfigFilter(&hcan1, &sFConfig1);
     
    txmsg3.StdId = 0x300;
    txmsg3.ExtId = 0x01;
    txmsg3.IDE = CAN_ID_STD;
    txmsg3.RTR = CAN_RTR_DATA;
    txmsg3.DLC = 8;
       txmsg3.Data[0]=5;
    txmsg3.Data[1]=3;
    txmsg3.Data[2]=5;
    txmsg3.Data[3]=3;
    txmsg3.Data[4]=5;
    txmsg3.Data[5]=3;
   txmsg3.Data[6]=5;
    txmsg3.Data[7]=3;
    hcan3.pTxMsg = &txmsg3; 
    hcan3.pRxMsg = &rxmsg3; 
    
     sFConfig3.FilterNumber = 0; // [0..13]
     sFConfig3.FilterMode = CAN_FILTERMODE_IDMASK;
     sFConfig3.FilterScale = CAN_FILTERSCALE_32BIT;
     sFConfig3.FilterIdHigh = 0; //COMP
     sFConfig3.FilterIdLow = 0;
     sFConfig3.FilterMaskIdHigh = 0; //MASK
     sFConfig3.FilterMaskIdLow = 0;
     sFConfig3.FilterFIFOAssignment = 0;
     sFConfig3.FilterActivation = ENABLE;
     sFConfig3.BankNumber = 14;

     HAL_CAN_ConfigFilter(&hcan3, &sFConfig3);
     

	old_uwTick = uwTick;
         
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_CAN_Receive(&hcan1, CAN_FIFO0,10);   
      HAL_CAN_Transmit(&hcan3,10);
    //msg=hcan1.pRxMsg->Data[0];
    //msg2=hcan1.pRxMsg->Data[1];
    

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_13TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
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
  hcan3.Init.Prescaler = 6;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SJW = CAN_SJW_1TQ;
  hcan3.Init.BS1 = CAN_BS1_13TQ;
  hcan3.Init.BS2 = CAN_BS2_2TQ;
  hcan3.Init.TTCM = DISABLE;
  hcan3.Init.ABOM = DISABLE;
  hcan3.Init.AWUM = DISABLE;
  hcan3.Init.NART = DISABLE;
  hcan3.Init.RFLM = DISABLE;
  hcan3.Init.TXFP = DISABLE;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

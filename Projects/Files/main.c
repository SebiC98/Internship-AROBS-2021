/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_intern.h"
#include "i2c_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LCD_REFRESH_INTERVAL 1000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan3;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

CAN_FilterConfTypeDef sFConfig1;
CanTxMsgTypeDef txmsg1;
CanRxMsgTypeDef rxmsg1;

char string[1];


static uint32_t rpm_feedback=0;
static uint32_t speed_feedback=0;
static uint8_t gear_feedback=0;
static uint8_t mod_feedback=0;

static uint8_t LB_feedback = 0;
static uint8_t HB_feedback = 0;
static uint8_t FL_feedback = 0;
static uint8_t FR_feedback = 0;
static uint8_t EM_feedback = 0;
static uint8_t FF_feedback = 0;

extern uint32_t uwTick;
uint32_t old_uwTick;
CAN_FilterConfTypeDef sFConfig3;
CanTxMsgTypeDef txmsg3;
CanRxMsgTypeDef rxmsg3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim14)
{
        lcd_init();
      //  lcd_send_cmd(0x01); //clear all
        //1st line
        lcd_send_cmd(0x80 | 0x00); 
	lcd_send_string ("RPM:");
        lcd_send_cmd(0x80 | 0x04); 
	lcd_send_data(rpm_feedback);
        
        //2st line
        lcd_send_cmd(0x80 | 0x45); 
	lcd_send_string ("disehtyhtie");
        
        //3st line
        lcd_send_cmd(0x80 | 0x1B); 
	lcd_send_string ("Uyht");
        
        //4st line
        lcd_send_cmd(0x80 | 0x5C); 
	lcd_send_string ("205rhy");


}*/

void first_line()
{
  lcd_init2();
         //1st line
        lcd_send_cmd(0x80 | 0x00);
	lcd_send_string("RPM:");
        LCD_itoa(rpm_feedback, string);
        lcd_send_cmd(0x80 | 0x04);
        lcd_send_string(string);
        lcd_send_cmd(0x80 | 0x0B);
        lcd_send_string("SPEED:");
        LCD_itoa(speed_feedback, string);
        lcd_send_cmd(0x80 | 0x011);
        lcd_send_string(string);
}

void second_line()
{
  //lcd_init2();
//2st line
        lcd_send_cmd(0x80 | 0x41);
	lcd_send_string("GEAR:");
        LCD_itoa(gear_feedback, string);
        lcd_send_cmd(0x80 | 0x46);
        lcd_send_string(string);
        lcd_send_cmd(0x80 | 0x49);
        lcd_send_string("MOD:");
        lcd_send_cmd(0x80 | 0x04D);
        if(mod_feedback == 1u)
        {
          lcd_send_string("ECO");
        }
        else if(mod_feedback == 2u)
        {
          lcd_send_string("NORMAL");
        }
        else if(mod_feedback == 3u)
        {
          lcd_send_string("SPORT");
        }
        else if(mod_feedback == 4u)
        {
          lcd_send_string("RESET");
        }
        else 
        {
          lcd_send_string("SPORT");
        }
}

void third_line()
{
  //lcd_init2();
        //3st line
        lcd_send_cmd(0x80 | 0x16);
	lcd_send_string("LB:");
        LCD_itoa(LB_feedback, string);
        lcd_send_cmd(0x80 | 0x19);
        lcd_send_string(string);
        lcd_send_cmd(0x80 | 0x1C);
        lcd_send_string("HB:");
        LCD_itoa(HB_feedback, string);
        lcd_send_cmd(0x80 | 0x01F);
        lcd_send_string(string);
        lcd_send_cmd(0x80 | 0x22);
        lcd_send_string("FF:");
        LCD_itoa(FF_feedback, string);
        lcd_send_cmd(0x80 | 0x025);
        lcd_send_string(string);
}

void fourth_line()
{
  //lcd_init2();
        //4st line
        lcd_send_cmd(0x80 | 0x56);
	lcd_send_string("FL:");
        LCD_itoa(FL_feedback, string);
        lcd_send_cmd(0x80 | 0x59);
        lcd_send_string(string);
        lcd_send_cmd(0x80 | 0x5C);
        lcd_send_string("FR:");
        LCD_itoa(FR_feedback, string);
        lcd_send_cmd(0x80 | 0x05F);
        lcd_send_string(string);
        lcd_send_cmd(0x80 | 0x62);
        lcd_send_string("EM:");
        LCD_itoa(EM_feedback, string);
        lcd_send_cmd(0x80 | 0x065);
        lcd_send_string(string);
}

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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN3_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
    	    //   lcd_send_cmd (0x01);
        lcd_init();
        
        lcd_send_cmd(0x80 | 0x05); //1st line
	lcd_send_string ("Lucrare de");
        lcd_send_cmd(0x80 | 0x45); //2st line
	lcd_send_string ("disertatie");
        lcd_send_cmd(0x80 | 0x1B); //3st line
	lcd_send_string ("UTC-N");
        lcd_send_cmd(0x80 | 0x5C); //4st line
	lcd_send_string ("2019");
        
        
    txmsg1.StdId = 0x100;
    txmsg1.ExtId = 0x01;
    txmsg1.IDE = CAN_ID_STD;
    txmsg1.RTR = CAN_RTR_DATA;
    txmsg1.DLC = 8;
     
    hcan1.pTxMsg = &txmsg1; 
    hcan1.pRxMsg = &rxmsg1; 
    
     sFConfig1.FilterNumber = 0; // [0..13]
     sFConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
     sFConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
     sFConfig1.FilterIdHigh = 0x200<<5; //COMP
     sFConfig1.FilterIdLow = 0;
     sFConfig1.FilterMaskIdHigh = 0xFFFF; //MASK
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
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      /*      lcd_send_cmd(0x80 | 0x50); //4st line
	lcd_send_data(rpm_feedback);
    */
    HAL_CAN_Receive(&hcan1, CAN_FIFO0,10);
    
    if(hcan1.pRxMsg->Data[0] <5)
    {
      txmsg3.StdId = 0x300;
    txmsg3.ExtId = 0x01;
    txmsg3.IDE = CAN_ID_STD;
    txmsg3.RTR = CAN_RTR_DATA;
    txmsg3.DLC = 4;
    HAL_CAN_ConfigFilter(&hcan3, &sFConfig3);
      hcan3.pTxMsg->Data[0] = hcan1.pRxMsg->Data[0];
    }
    if((hcan1.pRxMsg->Data[0] >4)&&(hcan1.pRxMsg->Data[0] <17))
    {
      txmsg3.StdId = 0x301;
    txmsg3.ExtId = 0x01;
    txmsg3.IDE = CAN_ID_STD;
    txmsg3.RTR = CAN_RTR_DATA;
    txmsg3.DLC = 1;
    HAL_CAN_ConfigFilter(&hcan3, &sFConfig3);
      hcan3.pTxMsg->Data[0] = hcan1.pRxMsg->Data[0];
    }
    
    HAL_CAN_Transmit(&hcan3,10);
    
    
    HAL_CAN_Receive_IT(&hcan3, CAN_FIFO0);
    
    if(hcan3.pRxMsg->StdId == 0x101)
    {
      rpm_feedback = CAN_intern_Read(&hcan3, 0, 0, 13);
      speed_feedback = CAN_intern_Read(&hcan3, 1, 5, 9); 
      gear_feedback = CAN_intern_Read(&hcan3, 2, 6, 3);
      mod_feedback = CAN_intern_Read(&hcan3,3 , 1, 2);       
    }
    if(hcan3.pRxMsg->StdId == 0x102)
    {
      LB_feedback = CAN_intern_Read(&hcan3, 0, 0, 1);
      HB_feedback = CAN_intern_Read(&hcan3, 0, 1, 1); 
      FL_feedback = CAN_intern_Read(&hcan3, 0, 2, 1);
      FR_feedback = CAN_intern_Read(&hcan3, 0, 3, 1);  
      EM_feedback = CAN_intern_Read(&hcan3, 0, 4, 1);
      FF_feedback = CAN_intern_Read(&hcan3, 0, 5, 1);
    }
    
    if((uwTick - old_uwTick) > LCD_REFRESH_INTERVAL)
    {
      first_line();
      second_line();
      third_line();
      fourth_line();
      
      old_uwTick = uwTick;
    }
    
        

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 499;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 47999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
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

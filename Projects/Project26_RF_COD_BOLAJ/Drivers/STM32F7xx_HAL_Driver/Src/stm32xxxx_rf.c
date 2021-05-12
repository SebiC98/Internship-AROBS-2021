/**
  ******************************************************************************
  * @file    stm32xxxx_rf.h
  * @author  Benedek Balazs
  * @brief   Header file of RF module
  ******************************************************************************
  * @attention
  *
  * This file is written under the direction of Arobs Transylvanian Software
  *           during automotive internship in 2021.
  *
  ******************************************************************************
**/

/* Includes --------------------------------------------------------------------- */
#include "stm32xxxx_rf.h"
#include "stm32f7xx_hal.h"

uint32_t tick_start = 0;

/**
  * @brief  Initiate and transmit a RF frame message.
  * @param  rf: pointer to a RF_HandleTypeDef structure that contains
  *         the configuration information for the specified RF.  
  * @param  Timeout: Timeout duration.
  * @retval RF status
  */

RF_StatusTypeDef RF_SetBit(uint8_t bit){

  switch(bit){
    case HIGH   : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)  ; break;
    case LOW    : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); break;
    default     : return RF_ERROR;
  }

  /* Get tick */
  tick_start = HAL_GetTick();

  /* Delay for data transmission */
  while(1){
   if(HAL_GetTick() - tick_start > DELAY) 
	break;
  //
}
  
  /* Return function status */
  return RF_OK;
  
}

RF_StatusTypeDef RF_Transmit(RF_HandleTypeDef* rf, uint32_t Timeout)
{
	/* Get tick */
	tick_start = HAL_GetTick();

	/* Set up the data field */
	for(int i = 0; i < 8; ++i){
       
		if(RF_SetBit(rf->pTxMsg->Data[i]) == RF_ERROR)
			return RF_ERROR;
          RF_SetBit(rf->pTxMsg->Data[i]);
      
	}

	/* Return function status if it is TIMEOUT*/
	if(HAL_GetTick() - tick_start > Timeout)
		return RF_TIMEOUT;
//
	/* Return function status */
	return RF_OK;
}
RF_StatusTypeDef RF_Receive (RF_HandleTypeDef *rf, uint32_t Timeout){
  /* Get tick */
	tick_start = HAL_GetTick();
  /* Set up the data field */
	for(int i = 0; i < 8; ++i){
        
	rf->pRxMsg->Data[i]=HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
	}
  
  
  return RF_OK;}
/**
  * @brief  Set a single bit from the data being transmitted.
  * @param  bit: a bit from the data being transmitted.
  * @retval RF status
  */


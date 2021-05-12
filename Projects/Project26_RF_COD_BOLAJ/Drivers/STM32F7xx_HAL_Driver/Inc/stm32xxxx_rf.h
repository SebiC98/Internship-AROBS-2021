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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32xxxx_RF_H
#define __STM32xxxx_RF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>

#define HIGH 1
#define LOW  0
#define DELAY 0.000000000000001 // 10^(-12) microseconds

typedef enum
{
  RF_OK       = 0x00,
  RF_ERROR    = 0x01,
  RF_TIMEOUT  = 0x02
} RF_StatusTypeDef;

/**
  * @brief  RF Tx message structure definition
  */
typedef struct RF_Tx_message_structure_definition
{
  uint16_t ID;    	/*!< Specifies the ID of the transmitter.
                             This parameter must be a number between 0 and 0x7FF */

  uint16_t Data[8];   	/*!< Contains the data to be transmitted.
                             This parameter must be a number between Min_Data = 0 and Max_Data = 0xFFFF */

}RF_TxMsgTypeDef;

/**
  * @brief  RF Rx message structure definition
  */
typedef struct RF_Rx_message_structure_definition
{
  uint16_t ID;       	/*!< Specifies the received ID of the transmitter.
                             This parameter must be a number between 0 and 0x7FF */

  uint16_t Data[8];  	/*!< Contains the data to be received.
                             This parameter must be a number between 0 and 0xFFFF */

}RF_RxMsgTypeDef;


/**
  * @brief  RF handle Structure definition
  */
typedef struct
{ 
  RF_TxMsgTypeDef*		pTxMsg;     /*!< Pointer to transmit  structure */
  RF_RxMsgTypeDef*		pRxMsg;     /*!< Pointer to reception structure */

}RF_HandleTypeDef;


RF_StatusTypeDef RF_Transmit(RF_HandleTypeDef *rf, uint32_t Timeout);
RF_StatusTypeDef RF_Receive (RF_HandleTypeDef *rf, uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif /* __STM32L4xx_CAN_H */


/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "string.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* SPI TIMEOUT Value*/
#define TIMEOUT_VAL 60
char Byte[30]; //array to print RTD resistance

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
void CS_ENABLE(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_Pin)
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

void CS_DISABLE(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_Pin)
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/* Queue Handles -------------------------------------------------------------*/
xQueueHandle Global_Queue_Data_H = 0;
xQueueHandle Global_Queue_Data_L = 0;
uint8_t Data_receive_h;
uint8_t Data_receive_l;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);

void receive_task(void *pvArgs);
void send_task(void *pvArgs);
void digitalread_task(void *pvArgs);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();

  Global_Queue_Data_H = xQueueCreate(1,sizeof(uint8_t));
  Global_Queue_Data_L = xQueueCreate(1,sizeof(uint8_t));
  /* USER CODE BEGIN 2 */


  xTaskCreate(receive_task, "Receiver task", 128, NULL, 1, NULL);


  /* Start scheduler */
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */


  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

uint16_t maxim_spi_16bit_transfer(uint16_t u_mosi)
	/**
* \brief				SPI 16-bit transfer
*
* \param[in]		u_mosi: SPI transmit data
*
* \retval				SPI receive data
*/
{
	uint16_t u_miso;
	CS_ENABLE(GPIOB, NCS12_Pin);	//NCS = 0
	//SPI_I2S_SendData(SPI2, u_mosi);
	HAL_SPI_Transmit(&hspi2, &u_mosi, 2, TIMEOUT_VAL);
	/* Wait for SPI2 data reception */
	//while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	HAL_SPI_Receive(&hspi2, &u_miso, 2, TIMEOUT_VAL);
	//u_miso=SPI_I2S_ReceiveData(SPI2);
	CS_DISABLE(GPIOB, NCS12_Pin);	//NCS = 1
	return u_miso;
}

void receive_task(void *pvArgs) {

	uint8_t data_holder;
	uint8_t data_to_send;
	uint8_t ok_to_send;
	HAL_StatusTypeDef TransmitReturn;
	uint32_t u_miso;
	uint8_t u_byte_h = 0;
	uint8_t u_byte_l = 0;
	uint8_t u_mosi = 0x00;

	for(;;) {

		HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);
  		if(HAL_CAN_Receive(&hcan, CAN_FIFO0, 1000) != HAL_OK) { //Try to receive

  			HAL_UART_Transmit(&huart1, (uint8_t *)"Receiving error!!", strlen("Receiving error!!"), HAL_MAX_DELAY);
  			HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);

  		} else { //Succes!

  			data_holder = hcan.pRxMsg->Data[0];
  			//char buff[20];
  			if(data_holder == 255){
  				ok_to_send = 255;
  			}
  			else if( data_holder == 0xAA){
  				ok_to_send = 0;
  			}
  			while (ok_to_send == 255) {


  				///Digital Read CODE
  				CS_ENABLE(GPIOB, NCS12_Pin);	//NCS = 0
  				//HAL_SPI_Transmit(&hspi2, &u_mosi, 2, TIMEOUT_VAL);
  				HAL_SPI_Receive(&hspi2, &u_miso, 2, TIMEOUT_VAL);
  				CS_DISABLE(GPIOB, NCS12_Pin);
  				u_byte_h = u_miso>>24;
  				u_byte_l = u_miso>>8;
  				//////



  				hcan.pTxMsg->Data[0] = 01;
  				hcan.pTxMsg->Data[1] = u_byte_h;
  				hcan.pTxMsg->Data[2] = u_byte_l;
  				//hcan.pTxMsg->Data[1] = Data_receive_h;
  				//hcan.pTxMsg->Data[2] = Data_receive_l;
  				hcan.pTxMsg->Data[3] = 0x00;
  				hcan.pTxMsg->Data[4] = 0x00;
  				hcan.pTxMsg->Data[5] = 0x00;
  				hcan.pTxMsg->Data[6] = 0x00;
  				hcan.pTxMsg->Data[7] = 0x00;
  				//hcan.pTxMsg->Data[3] = m;

  				HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);
  		  		if(HAL_CAN_Receive(&hcan, CAN_FIFO0, 5) != HAL_OK) { //Try to receive

  		  			HAL_UART_Transmit(&huart1, (uint8_t *)"Receiving error", strlen("Receiving error"), HAL_MAX_DELAY);
  		  			HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);

  		  		} else {
  	  			data_holder = hcan.pRxMsg->Data[0];
  	  			//char buff[20];
  	  			if(data_holder == 255){
  	  				ok_to_send = 255;
  	  			}
  	  			else if( data_holder == 0xAA){
  	  				ok_to_send = 0;
  	  				break;
  	  			}
  				//vTaskDelay(pdMS_TO_TICKS(100));
  		  		}

  				TransmitReturn = HAL_CAN_Transmit(&hcan, 5); //Try to transmit and get result

  				if (TransmitReturn == HAL_ERROR) { //We got an error
  				  	/* Transmitting Error */
  				  	HAL_UART_Transmit(&huart1, (uint8_t*)"Failed to transmit", strlen("Failed to receive"), HAL_MAX_DELAY);
  				  	HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);

  				} else if((TransmitReturn == HAL_TIMEOUT)){ //Timed out
  				  	HAL_UART_Transmit(&huart1, (uint8_t*)"Timeout", strlen("Timeout"), HAL_MAX_DELAY);
  				  	HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);

  				} else if((TransmitReturn == HAL_OK)){ //Everything worked
  				  	HAL_UART_Transmit(&huart1, (uint8_t*)"Transmitting: ", strlen("Transmitting: "), HAL_MAX_DELAY);

  				  	data_to_send = hcan.pTxMsg->Data[3];

  				  	char trans_buff[20];
  				  	sprintf(trans_buff, "%i", data_to_send);
  				  	HAL_UART_Transmit(&huart1, (uint8_t*)trans_buff, strlen(trans_buff), HAL_MAX_DELAY);
  				  	HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);

  				}
  				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

  				}

  		}

	}
}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{
	CAN_FilterConfTypeDef  sFilterConfig;

  static CanTxMsgTypeDef        TxMessage;
  static CanRxMsgTypeDef        RxMessage;

  hcan.pTxMsg = &TxMessage;
  hcan.pRxMsg = &RxMessage;

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 2; /// 2 is for 500Kbit/s and 8 is for 125Kbit/s baud rate
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_2TQ;
  hcan.Init.BS2 = CAN_BS2_1TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  hcan.pTxMsg->StdId = 0x005;
  hcan.pTxMsg->IDE   = CAN_ID_STD;//values defined in different hal libraries
  hcan.pTxMsg->RTR   = CAN_RTR_DATA;//values defined in different hal libraries
  hcan.pTxMsg->DLC   = 8;//1-9 // how many data frames in CAN

  int filter_id = 0x00000005;
  int filter_mask = 0x1FFFFFFF;

  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = ((filter_id << 5)  | (filter_id >> (32 - 5))) & 0xFFFF; // STID[10:0] & EXTID[17:13]
  sFilterConfig.FilterIdLow = (filter_id >> (11 - 3)) & 0xFFF8; // EXID[12:5] & 3 Reserved bits
  sFilterConfig.FilterMaskIdHigh = ((filter_mask << 5)  | (filter_mask >> (32 - 5))) & 0xFFFF;
  sFilterConfig.FilterMaskIdLow = (filter_mask >> (11 - 3)) & 0xFFF8;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 500000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS4_Pin|CS3_Pin|CS9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|CS8_Pin|CS7_Pin|CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS4_Pin CS3_Pin CS9_Pin */
  GPIO_InitStruct.Pin = CS4_Pin|CS3_Pin|CS9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin CS8_Pin CS7_Pin CS2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|CS8_Pin|CS7_Pin|CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Pin CS6_Pin */
  GPIO_InitStruct.Pin = CS1_Pin|CS6_Pin|NCS12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS5_Pin CS10_Pin */
  GPIO_InitStruct.Pin = CS5_Pin|CS10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

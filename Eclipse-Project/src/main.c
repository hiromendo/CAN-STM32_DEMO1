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
#include "string.h"   // added from TempCode
#include <math.h>     // added from TempCode

/* USER CODE BEGIN Includes */

//////From Temp Code
#define LEDred_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()

/* USER CODE BEGIN Includes */
/* Macros to enable & disable CS pin */
//#define CS_ENABLE		do { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); } while(0);
//#define CS_DISABLE		do { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); } while(0);

/* SPI TIMEOUT Value*/
#define TIMEOUT_VAL 60

// The value of the Rref resistor in Ohm
#define RREF 470000
//#define RREF 15000

/* Read Register Address */
#define REG_CONFIG                  0x00
#define REG_RTD_MSB                 0x01
#define REG_RTD_LSB                 0x02
#define REG_HIGH_FAULT_THR_MSB      0x03
#define REG_HIGH_FAULT_THR_LSB      0x04
#define REG_LOW_FAULT_THR_MSB       0x05
#define REG_LOW_FAULT_THR_LSB       0x06
#define REG_FAULT_STATUS            0x07
#define WR(reg)                     ( (reg) | 0x80 )
///// FROM Temp Code

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi1;  // added from TempCode
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* From Temp Code ---------------------------------------------------------*/
struct __attribute__((packed)) var_max31865
{
  uint16_t rtd_res_raw;			// RTD IC raw resistance register
  uint8_t  status;					// RTD status - full status code
  uint8_t  conf_reg;				// Configuration register readout
  uint16_t  HFT_val;				// High fault threshold register readout
  uint16_t  LFT_val;				// Low fault threshold register readout
};

struct var_max31865 rtd_data;
uint8_t read_addr = 0x00; //Read address of Configuration register

/*variables of resistance and temperature*/
double resistanceRTD;
double tmp;

/*register to initiate SPI*/
uint8_t config_reg_write[] = {WR(REG_CONFIG), 0xC2};

/*arrays to print values in for resistance and temperature*/
char Rrtd[60]; //array to print RTD resistance
char Trtd[60]; //array to print RTD temperature
char Stop[60]; //indicates all is read
char SlaveOutput[80]; //bits from SPI slave
char *msg = "Initiating Temperature measurement\n\r";

/*ChipSelect pins and ports*/
GPIO_TypeDef* CS_GPIO_Port[10]={CS1_GPIO_Port, CS2_GPIO_Port,CS3_GPIO_Port,CS4_GPIO_Port,CS5_GPIO_Port,CS6_GPIO_Port,CS7_GPIO_Port,CS8_GPIO_Port,CS9_GPIO_Port,CS10_GPIO_Port};
uint16_t CS_Pin[10]={CS1_Pin,CS2_Pin,CS3_Pin,CS4_Pin,CS5_Pin,CS6_Pin,CS7_Pin,CS8_Pin,CS9_Pin,CS10_Pin};
unsigned short int TemperatureValues_K[10]={0,0,0,0,0,0,0,0,0,0};
unsigned short int ResistanceValues_K[10]={0,0,0,0,0,0,0,0,0,0};

/*SPI LEDs*/
uint8_t lightAllLeds [28] ={0x96, 0xDF, 0xFF, 0xFF, 0x20, 0xFF, 0x20, 0xFF, 0x20, 0xFF, 0x20, 0xFF, 0x20, 0xFF, 0x20, 0xFF, 0x20, 0xFF, 0x20, 0xFF, 0x20, 0xFF, 0x20, 0xFF, 0x20, 0xFF,0x20,0xFF};
uint8_t lightNoLeds [28] ={0x96, 0xDF, 0xFF, 0xFF,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t LEDinit [28] ={0x96, 0xDF, 0xFF, 0xFF,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


		char MSG[30];
		char debug[30];
/* From Temp Code---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);   /// From Temp Code
//static GPIO_InitTypeDef  GPIO_InitStruct; /// From Temp Code

void receive_task(void *pvArgs);
void send_task(void *pvArgs);
//void Read_Temperature(void *pvArgs); /// From Temp Code

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/// From Temp Code
void CS_ENABLE(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_Pin);
void CS_DISABLE(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_Pin);
void configureSPI(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_Pin);
void MAX31865_full_read(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_Pin, int LED, uint8_t *LEDinit,int CSnumber);
/// From Temp Code
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

  /* -1- Enable GPIO Clock (to be able to program the configuration registers) */
  /// From Temp Code
  // LEDred_GPIO_CLK_ENABLE();
   /// From Temp Code

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();   /// from Temp Code MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  ///// From Temp Code
//  /* -2- Configure IO in output push-pull mode to drive external LEDs */
//    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull  = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//
//    GPIO_InitStruct.Pin = GPIO_PIN_0;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    /* USER CODE BEGIN 2 */
//
//  	HAL_UART_Transmit(&huart1, (uint8_t*)"\n\rConfigure SPI2", strlen("\n\rConfigure SPI2"), HAL_MAX_DELAY);
//
//  for(int conf=0;conf< 10;conf++)
//  	{
//  	configureSPI(CS_GPIO_Port[conf],CS_Pin[conf]);
//  	//HAL_Delay(50);
//
//  	}
//  	HAL_UART_Transmit(&huart1, (uint8_t*)"\n\rConfiguration done\n\r", strlen("\n\rConfiguration done\n\r"), HAL_MAX_DELAY);
//
//  	// give the sensor time to set up
//    	HAL_SPI_Transmit(&hspi1, lightAllLeds, 28, 10);
//    	HAL_Delay(1000);

 /// From TempCode

  xTaskCreate(receive_task, "Receiver task", 128, NULL,1, NULL);
 // xTaskCreate(send_task, "Sender task", 128, NULL, 1, NULL);
  //xTaskCreate(Read_Temperature, "Read Temperature", 128, NULL, 1, NULL);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

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
/* FROM TEMP CODE */
void Read_Temperature(void *pvArgs) {


while(1) {
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);


    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    HAL_Delay(50);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    HAL_Delay(50);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
    HAL_Delay(50);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);

/* USER CODE BEGIN 3 */
	for(int read= 9;read>=0;read--)
    {
    MAX31865_full_read(CS_GPIO_Port[read],CS_Pin[read],read,LEDinit,read);

    }
	HAL_SPI_Transmit(&hspi1, LEDinit, 28, 10);

	sprintf(Stop, "\n\rReading done\n\r");
	HAL_UART_Transmit(&huart1, (uint8_t *)Stop, 30, TIMEOUT_VAL);

	//HAL_Delay(500);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void CS_ENABLE(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_Pin)
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

void CS_DISABLE(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_Pin)
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/* Function to configure and initiate SPI */

void configureSPI(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_Pin)
{
 /* (1) : SPI Transmit, write to config reg on address 0x80 */
  // Step(1): Bring the CS pin low to activate the slave device
  CS_ENABLE(CS_GPIO_Port, CS_Pin);
  HAL_Delay(10); //This delay is very important in the case of STM32F334 in order to work with MAX31865
  // Step(2): Transmit config reg address  & data
  HAL_SPI_Transmit(&hspi2, &config_reg_write[0], 1, TIMEOUT_VAL);
  HAL_SPI_Transmit(&hspi2, &config_reg_write[1], 1, TIMEOUT_VAL);
  // Step(3): Bring the CS pin high again
  CS_DISABLE(CS_GPIO_Port, CS_Pin);
}

/* Function to unpack and store MAX31865 data */

void MAX31865_full_read(GPIO_TypeDef* CS_GPIO_Port, uint16_t CS_Pin, int LED, uint8_t *LEDinit,int CSnumber)
{
	uint8_t read_data[8]={0,0,0,0,0,0,0,0}; //variable to store the contents of the registers
	uint8_t i = 0; //loop variable

	// Step(1): Bring the CS pin low to activate the slave device
	CS_ENABLE(CS_GPIO_Port, CS_Pin);
	// Step(2): Transmit config reg address telling IC that we want to 'read' and start at register 0
		HAL_SPI_Transmit(&hspi2, &read_addr, 1, TIMEOUT_VAL);
	/* Step (3): Receive the first 8 bits (Config reg data) */
	for(i = 0; i < 8; i++)
	{
		HAL_SPI_Receive(&hspi2, &read_data[i], 1, TIMEOUT_VAL);
	}
	// Step(4): Bring the CS pin high again
	CS_DISABLE( CS_GPIO_Port, CS_Pin);
	/* Step (5): Store the data read from the sensor */
	rtd_data.conf_reg = read_data[0]; //Store config reg data
	rtd_data.rtd_res_raw = ((read_data[1] << 8) | read_data[2]) >> 1; // Store rtd_res_raw
	rtd_data.HFT_val = ((read_data[3] << 8) | read_data[4]) >> 1; // Store HFT_val
	rtd_data.LFT_val = (read_data[5] << 8) | read_data[6]; // Store LFT_val
	rtd_data.status = read_data[7]; //Store fault status reg data

	sprintf(SlaveOutput, "\n\r0 %u\n\r1 %u\n\r2 %u\n\r3 %u\n\r4 %u\n\r5 %u\n\r6 %u\n\r7 %u\n\r", read_data[0],read_data[1],read_data[2],read_data[3],read_data[4],read_data[5],read_data[6],read_data[7]);
    HAL_UART_Transmit(&huart1, (uint8_t *)SlaveOutput, 80, TIMEOUT_VAL);

    HAL_UART_Transmit(&huart1, (uint8_t *)"RUNS TIL HERE",strlen("RUNS TIL HERE"), TIMEOUT_VAL);

	/*Enable LED if thermistor is connected*/
	//LED=9-LED;
	if (rtd_data.status < 128 ||(rtd_data.rtd_res_raw < 32767 && rtd_data.rtd_res_raw > 0))
		{
	*(LEDinit+(27-LED*2))= 0xFF;
	*(LEDinit+(26-LED*2))= 0x20;

	//for lighting up one after another
	//HAL_SPI_Transmit(&hspi1, LEDinit, 28, 10);
		}
	else if (rtd_data.status >= 128||(rtd_data.rtd_res_raw >= 32767 && rtd_data.rtd_res_raw == 0))
		{
	*(LEDinit+(27-LED*2)) = 0;
	*(LEDinit+(26-LED*2)) = 0;

	//for lighting up one after another
	//HAL_SPI_Transmit(&hspi1, LEDinit, 28, 10);
		}

  /* calculate RTD resistance */
	//5143.92 as offset for 470k resistor
	//CSnumber=CSnumber+1;
	    resistanceRTD = /*5143.92+*/((double)rtd_data.rtd_res_raw * RREF) / 32767; // Replace 4000 by 400 for PT100
		sprintf(Rrtd, "\n\rCS%i: \n\rRrtd = %0.2f\n\rRAW = %u\n\r", CSnumber+1, resistanceRTD,rtd_data.rtd_res_raw);
		HAL_UART_Transmit(&huart1, (uint8_t *)Rrtd, 60, TIMEOUT_VAL); // print RTD resistance

		//sprintf(Rrtd, "RAW = %u\n\r", rtd_data.rtd_res_raw);
		//HAL_UART_Transmit(&huart1, (uint8_t *)Rrtd, 60, TIMEOUT_VAL); // print RTD resistance

	/* calculate RTD temperature (simple calc, +/- 2 deg C from -100C to 100C) */
    /* CALCULATION OF TEMP MUST BE ADJUSTED TO RTD  */
  //  tmp = ((double)rtd_data.rtd_res_raw / 32) - 256;
	 tmp=1/(((log(/*(double)rtd_data.rtd_res_raw*/ resistanceRTD / 10000))/3435)+(1/298.15));
	//tmp=tmp-273.15;
	// http://www.giangrandi.ch/electronics/ntc/ntc.shtml
	//http://www.carelparts.com/manuals/ntc-specs.pdf page 8



		sprintf(Trtd, "Trtd = %0.2f\n\r", tmp);
    HAL_UART_Transmit(&huart1, (uint8_t *)Trtd, 60, TIMEOUT_VAL); // print RTD temperature

    /*transferring kelvin with 2 decimals to unsigned 16 bit*/

    tmp=tmp*100;
	resistanceRTD=resistanceRTD*100;
	*(TemperatureValues_K+9-CSnumber)=(unsigned short int)tmp;
	*(ResistanceValues_K+9-CSnumber)=(unsigned short int)resistanceRTD;

	sprintf(Trtd, "Temp in Array = %hu\n\r", TemperatureValues_K[9-CSnumber]);
	    HAL_UART_Transmit(&huart1, (uint8_t *)Trtd, 60, TIMEOUT_VAL); // print RTD temperature

	//HAL_Delay(200);
}
/* FROM TEMP CODE */

void receive_task(void *pvArgs) {

	uint8_t data_holder;
	uint8_t data_to_send;
	uint8_t ok_to_send;
	HAL_StatusTypeDef TransmitReturn;

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

  				int m;
  				for (m = 1; m < 11; m = m + 1){
  				hcan.pTxMsg->Data[0] = 00;
  				hcan.pTxMsg->Data[1] = m;
  				hcan.pTxMsg->Data[2] = 0x02;
  				hcan.pTxMsg->Data[3] = m;

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
  			//sprintf(buff, "%i", data_holder);
  		//	HAL_UART_Transmit(&huart1, (uint8_t*)"Receiving: ", strlen("Receiving: "), HAL_MAX_DELAY);
  		//	HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
  		//	HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);

  		}
  		//vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void send_task(void *pvArgs) {

	uint8_t data_to_send;
	HAL_StatusTypeDef TransmitReturn;



	hcan.pTxMsg->Data[0] = 01;
	hcan.pTxMsg->Data[1] = 0x01;
	hcan.pTxMsg->Data[2] = 0x1B;

	for(;;) {



		TransmitReturn = HAL_CAN_Transmit(&hcan, 1000); //Try to transmit and get result

		if (TransmitReturn == HAL_ERROR) { //We got an error
		  	/* Transmitting Error */
		  	HAL_UART_Transmit(&huart1, (uint8_t*)"Failed to transmit", strlen("Failed to receive"), HAL_MAX_DELAY);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);

		} else if((TransmitReturn == HAL_TIMEOUT)){ //Timed out
		  	HAL_UART_Transmit(&huart1, (uint8_t*)"Timeout", strlen("Timeout"), HAL_MAX_DELAY);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);

		} else if((TransmitReturn == HAL_OK)){ //Everything worked
		  	HAL_UART_Transmit(&huart1, (uint8_t*)"Transmitting: ", strlen("Transmitting: "), HAL_MAX_DELAY);

		  	data_to_send = hcan.pTxMsg->Data[0];

		  	char trans_buff[20];
		  	sprintf(trans_buff, "%i", data_to_send);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)trans_buff, strlen(trans_buff), HAL_MAX_DELAY);
		  	HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", strlen("\n\r"), HAL_MAX_DELAY);

		}
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		vTaskDelay(pdMS_TO_TICKS(1000));
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
  hcan.Init.Prescaler = 8; /// was 8 before for 125K bit rate 20 is supposed to be for 50K but that means we are at 4Mhz
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

  hcan.pTxMsg->StdId = 0x065;
  hcan.pTxMsg->IDE   = CAN_ID_STD;//values defined in different hal libraries
  hcan.pTxMsg->RTR   = CAN_RTR_DATA;//values defined in different hal libraries
  hcan.pTxMsg->DLC   = 4;//1-9

  int filter_id = 0x00000065;
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

/* SPI1 init function */

static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
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
  //HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS6_Pin, GPIO_PIN_RESET);// from working CAN DEMO
  HAL_GPIO_WritePin(GPIOB, CS1_Pin|CS5_Pin|CS6_Pin|CS10_Pin, GPIO_PIN_RESET); // from Temp code
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
  GPIO_InitStruct.Pin = CS1_Pin|CS6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS5_Pin CS10_Pin */  // THESE are from working CAN CODE
  //GPIO_InitStruct.Pin = CS5_Pin|CS10_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // THESE are from working CAN CODE
  /*Configure GPIO pins : CS1_Pin CS5_Pin CS6_Pin CS10_Pin */
   GPIO_InitStruct.Pin = CS1_Pin|CS5_Pin|CS6_Pin|CS10_Pin;  /// From Temp CODE
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

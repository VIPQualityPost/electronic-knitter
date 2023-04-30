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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KH930
#define K_CARRIAGE

#define END_LEFT		0xFF
#define END_RIGHT 		0

#define OFFSET_LEFT 	40
#define OFFSET_RIGHT	16

#define EOL_L_MIN
#define EOL_L_MAX
#define EOL_R_MIN
#define EOL_R_MAX

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t 	EOL_result[2];		/* ADC reading [0] left, [1] right */
uint16_t 	currentPattern;		/* current solenoid pattern */

uint8_t 	rowPattern[200]; 	/* 200 bytes (bed width) */
uint8_t 	currentDir;			/* Encoder direction */
uint8_t 	currentPosition; 	/* TIM1->CNT, use to sense movement */

char 		txBuf[64];			/* Used for USB serial messages */

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* Set the solenoids all off */
  GPIOB->ODR = 0x0000;

  /* Start the encoder and timers */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  /* Start the ADC */
  HAL_ADCEx_Calibration_Start(&hadc1);
  // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)EOL_result, 2);

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	if(currentPosition != TIM1->CNT){
		currentDir = TIM1->CNT - currentPosition;
		currentPosition = TIM1->CNT;

		advanceCarriage();
	}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief Write a pattern to the solenoid bank.
 * @param The 16-solenoid pattern to set.
 * @retval none
 * 
 * There are 16 solenoids, we can control which needles are selected based on the 
 * state of belt-phase and this solenoid bank.
*/
void writeSolenoids(uint16_t pattern){
  /* Store the pattern to write somewhere so we can read it later. */
  currentPattern = pattern;

  /* Need to byteswap because I swapped the banks on the connector. */
  pattern = (pattern << 8) | (pattern >>8);
  GPIOB->ODR = pattern;
}

void advanceCarriage(){

	/* Belt phase indicates which needle selector plate is active */
	uint8_t beltPhase = HAL_GPIO_ReadPin(BELTPHASE_GPIO_Port, BELTPHASE_Pin);
	uint8_t nextSolenoid;
	uint8_t stitchPosition;

	/* Moving to the right */
	if(currentDir){
		if(currentPosition <= (END_LEFT - OFFSET_LEFT)){
			stitchPosition = currentPosition - OFFSET_RIGHT;
			if(beltPhase){
				nextSolenoid = (currentPosition+8)%16;
			}
			else{
				nextSolenoid = (currentPosition)%16;
			}
		}
	}
	/* Moving to the left */
	else{
		if(currentPosition > OFFSET_RIGHT){
			stitchPosition = currentPosition - OFFSET_LEFT;
			if(beltPhase){
				nextSolenoid = (currentPosition)%16;
			}
			else{
				nextSolenoid = (currentPosition+8)%16;
			}
		}
	}

	if(0 <= stitchPosition && stitchPosition <=200){
		uint8_t stitchPixel = rowPattern[stitchPosition];
		HAL_GPIO_WritePin(GPIOB, nextSolenoid, stitchPixel);
	}
}

/**
 * @brief Timer overflow callback
 * @param Timer handle
 * @retval none
 * 
 * This function is called whenever a timer overflows. Need to sort by 
 * handle in order to define the custom behavior:  htim->Instance=TIMx
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
  if(htim->Instance==TIM2){
    HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
  }
  if(htim->Instance==TIM3){
    /* Kick off the ADC reading. */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)EOL_result, 2);
  }
}

/** 
 * @brief ADC conversion complete callback
 * @param ADC handle
 * @retval none
 * 
 * This function is called when the ADC finishes converting all the requested channels,
 * triggered by TIM3 interrupt 
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  sprintf(txBuf, "IN0: %i, IN1: %i\r\n", EOL_result[0], EOL_result[1]);
  CDC_Transmit_FS((uint8_t *)txBuf, strlen(txBuf));
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add their own implementation to report the HAL error return state */
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
  /* User can add their own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

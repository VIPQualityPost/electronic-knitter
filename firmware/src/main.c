/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @author		   : matei
 ******************************************************************************
*/

/* Includes */
#include "main.h"
#include "adc.h"
#include "ayab.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "math.h"
#include "usbd_cdc_if.h"

/* Defines */
#define HW_TEST 0
#define SWAP_SOLENOID 1

#define FW_AYAB 1 /* Enable if using the AYAB software. */

#define KH930
#define K_CARRIAGE

#define END_LEFT 0xFF
#define END_RIGHT 0

#define OFFSET_LEFT 40
#define OFFSET_RIGHT 16

#define EOL_L_MIN
#define EOL_L_MAX
#define EOL_R_MIN
#define EOL_R_MAX

/* Prototypes */
void SystemClock_Config(void);
void writeSolenoids(uint16_t);
void advanceCarriage(void);
void testEncoders(void);
void testEOL(void);
void testSolenoids(void);

/* Hardware peripherals */
uint16_t EOL_result[2];	 /* ADC reading [0] left, [1] right */

/* Machine variables */
uint8_t machineType;
uint8_t machineInitialized;
enum kmStates{RUN, STOP, INIT, WAIT, DONE};
enum kmStates machineState;

/* Encoder states */
uint8_t startDir;
uint8_t currentDir;
uint8_t currentPosition;
uint8_t lastBP;

/* Pattern information */
uint16_t 	currentPattern; /* current solenoid pattern */

uint8_t 	bitPattern[25];	/* The entire line to be knitted */

uint8_t 	startNeedle = 0xFF;
uint8_t 	stopNeedle = 0xFF;
uint8_t 	currentRow;
uint8_t 	lastRow;

/* General buffer*/
char 	debugString[128];		 /* Send to AYAB with txAYAB(debug) */

/* AYAB specific stuff*/
/* Firmware version information kept in ayab.h */
uint8_t *ayabBuf;
uint8_t lastCmd;
uint8_t crc8_cs;

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* Configure all the peripherals */
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_USB_DEVICE_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();

	/* Set the solenoids all off */
	GPIOB->ODR = 0x0000;

	/* Start the encoder and timers */
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	/* Start the ADC */
	HAL_ADCEx_Calibration_Start(&hadc1);

	while (1)
	{
		if (currentPosition != TIM1->CNT)
		{
			/* Check the direction and update the new position. */
			currentDir = TIM1->CNT - currentPosition;
			currentPosition = TIM1->CNT;

			uint8_t beltPhase = HAL_GPIO_ReadPin(BELTPHASE_GPIO_Port, BELTPHASE_Pin);

			if(lastBP != beltPhase){
				/* If the belt phase changes, then we need to byte swap. */
				lastBP = beltPhase;
				uint8_t swappedPattern = (currentPattern << 8) | (currentPattern >> 8);
				writeSolenoids(swappedPattern);
			}

			if (machineState == RUN)
			{
				advanceCarriage();
			}
		}
	}
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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_USB;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Write a pattern to the solenoid bank.
 * @param The 16-solenoid pattern to set.
 * @retval none
 *
 * There are 16 solenoids, we can control which needles are selected based on the
 * state of belt-phase and this solenoid bank.
 */
void writeSolenoids(uint16_t pattern)
{
	/* Store the pattern to write somewhere so we can read it later. */
	currentPattern = pattern;

	if (SWAP_SOLENOID)
	{
		/* Need to byteswap on rev1 boards because I swapped the banks on the connector. */
		pattern = (pattern << 8) | (pattern >> 8);
	}

	GPIOB->ODR = pattern;
}

/**
 * @brief Move the carriage one needle and set corresponding solenoid.
 * @param None
 * @retval None
 *
 * This function handles checking the belt phase when moving the carriage one direction or other.
 * After selecting the right solenoid and adjusting the position to make up for slop at end of lines,
 * it will set the new pattern on the solenoids.
 */
void advanceCarriage()
{
	writeSolenoids(0xFFFF);
}

/**
 * @brief Timer overflow callback
 * @param Timer handle
 * @retval none
 *
 * This function is called whenever a timer overflows. Need to sort by
 * handle in order to define the custom behavior:  htim->Instance=TIMx
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);

		if (lastCmd != *ayabBuf)
		{
			/* If there is a new command ID in the buffer, then we need to figure out what to do. */
			rxAYAB();
		}

		if (HW_TEST)
		{
			testEncoders();
			testEOL();
			testSolenoids();
		}
	}
	if (htim->Instance == TIM3)
	{
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
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	sprintf(debugString, "IN0: %i, IN1: %i\r\n", EOL_result[0], EOL_result[1]);
	CDC_Transmit_FS((uint8_t *)debugString, strlen(debugString));
}

void testSolenoids(void)
{
	/* Just increment the bank, cover 0x00 to 0xFF */
	GPIOB->ODR = GPIOB->ODR + 0x01;
}

void testEncoders(void)
{
	uint8_t beltPhase = HAL_GPIO_ReadPin(BELTPHASE_GPIO_Port, BELTPHASE_Pin);
	sprintf(debugString, "POSITION: %i, DIRECTION: %i, BELT PHASE: %i\r\n", (uint8_t)TIM1->CNT, currentDir, beltPhase);
	CDC_Transmit_FS((uint8_t *)debugString, strlen(debugString));
}

void testEOL(void)
{
	sprintf(debugString, "IN0: %i, IN1: %i\r\n", EOL_result[0], EOL_result[1]);
	CDC_Transmit_FS((uint8_t *)debugString, strlen(debugString));
}

// void calibrateEOL(void)
// {
// }

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

#ifdef USE_FULL_ASSERT
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

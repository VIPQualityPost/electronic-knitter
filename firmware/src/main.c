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
#define SWAP_SOLENOID 1 /* If on rev1 boards with solenoid banks swapped. */

#define FW_AYAB 1 /* Enable if using the AYAB software. */

#define KH930
#define NUMNEEDLES 200

#define END_LEFT 0
#define END_RIGHT 0xFF

#define OFFSET_LEFT 12
#define OFFSET_RIGHT 12

#define EOL_L_MIN 0xFF
#define EOL_L_MAX 0xFF
#define EOL_R_MIN 0xFF
#define EOL_R_MAX 0xFF

#define getBP() HAL_GPIO_ReadPin(BELTPHASE_GPIO_Port, BELTPHASE_Pin)

/* Prototypes */
void SystemClock_Config(void);
void writeSolenoids(uint16_t);
void advanceCarriage(void);
uint8_t carriageOffset(void);
void testEncoders(void);
void testEOL(void);
void testSolenoids(void);
void calibrateEOL(void);

/* Hardware peripherals */
uint16_t EOL_result[2]; /* ADC reading [0] left, [1] right */

/* Machine variables */
uint8_t machineType;
uint8_t machineInitialized;
enum machineStates
{
	INIT,
	RUN,
	TEST,
	CALIBRATE
} machineState;
enum carriageTypes
{
	UNKNOWN,
	K,
	L,
	G
} carriageType;

/* Encoder states */
enum {
	RIGHT, //0, TIM1 increment
	LEFT	//1, TIM1 decrement
}currentDir; /* Using shift in currentPosition to determine direction. */

uint8_t currentPosition; /* Compare with TIM1->CNT to see relative movement. */
uint8_t bpShift;			 /* belt phase position during EOL stop. */

/* Pattern information */
uint16_t currentPattern; /* current solenoid pattern */

uint8_t bitPattern[25]; 	/* The entire line to be knitted [25 bytes = 200 bits] 	*/
uint8_t currentByte;	/* Current byte from the bitPattern array. */
uint8_t startNeedle = 0xFF; /* Start needle for the pattern (subset of all needles) */
uint8_t stopNeedle = 0xFF;	/* End needle for the pattern (subset of all needles)	*/
uint8_t passStart;
uint8_t passStop;
uint8_t currentRow = 0; 	/* The row currently being worked by the machine. 		*/
uint8_t lastRow; 			/* Flag if the line received from AYAB is the last line of the pattern */

/* General buffer*/

/* AYAB specific stuff*/
/* Firmware version information kept in ayab.h */
uint8_t *ayabBuf; /* Pointer to AYAB serial buffer. */
uint8_t continuousReport;
uint8_t crc8_cs;  /* Checksum not used for anything now. */

char debugString[128]; /* Send to AYAB with txAYAB(debug) */

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
	machineState = INIT;
	carriageType = UNKNOWN;

	/* Start the encoder and timers */
	
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	/* Start the ADC */
	HAL_ADCEx_Calibration_Start(&hadc1);

	while (1)
	{
		switch (machineState)
		{
		case INIT:
			sprintf(debugString, "Waiting for pattern start signals...\r\n");
			txAYAB(debug);
			HAL_Delay(500);
			break;
		
		case RUN:
			if(currentPosition != TIM1->CNT){
				currentPosition = TIM1->CNT;
				currentDir = (TIM1->CR1 | TIM_CR1_DIR) >> TIM_CR1_DIR_Pos;
				// currentDir = TIM1->CNT - currentPosition; 

				advanceCarriage();

				if(continuousReport){
					txAYAB(indState);
				}
			}
			break;

		case CALIBRATE:
			calibrateEOL();
			HAL_Delay(100);
			break;

		case TEST:
			testEncoders();
			testEOL();
			testSolenoids();
			HAL_Delay(250);
			break;

		default:
			break;
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

	GPIOB->ODR = (uint32_t)pattern;
}

/**
 * @brief Move the carriage one needle and set corresponding solenoid.
 * @param None
 * @retval None
 *
 * This function handles checking to see if we should write a new byte to
 * the solenoid pattern bank. Need to handle offset slop ?
 */
void advanceCarriage()
{
	/** 
	 * machine orientation
	 * [EOL_L]	   [EOL_R]
	 * [byte 0 ][....][25]
	 * 0123456789......200
	*/
	uint8_t realPosition = currentPosition - carriageOffset();

	switch (currentDir)
	{
	case LEFT:
		/* Moving left */
		if (currentPosition <= OFFSET_RIGHT)
		{
			/* If our position is at the edge of a solenoid bank then we can move to the next byte in bitPattern */
			if (currentPosition % 16 == 0)
			{
				currentByte--;

			}
		}
		break;

	case RIGHT:
		/* Moving right */
		if (currentPosition >= OFFSET_LEFT)
		{
			if (currentPosition % 16 == 0)
			{	
				currentByte++;
			}
		}
		break;
	}

	uint16_t newPattern = bitPattern[currentByte];

	if(bpShift){
		newPattern = (newPattern << 8) | (newPattern >> 8);
	}
	writeSolenoids(newPattern);

	if(currentPosition == startNeedle){
		passStart = 1;
	}
	else if(currentPosition == stopNeedle){
		passStop =1;
	}

	if(passStart && passStop){
		/* We passed all the active needles so we can ask for the new line. */
		txAYAB(reqLine);

		/* Clear the progress flags. */
		passStart = 0;
		passStop = 0;
	}

	// Start a read so the next time we move we know if we are in front of a sensor.
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)EOL_result, 2);
}

/**
 * @brief 
 * @param
 * @retval 
 * 
 * 
*/
uint8_t carriageOffset(void)
{
	switch (currentDir)
	{
	case LEFT:
		/* moving left */

		if (carriageType == G)
		{
			return 8;
		}
		else
		{
			return 40;
		}
		break;

	case RIGHT:
		/* moving right */

		if (carriageType == G)
		{
			return 32;
		}
		else
		{
			return 16;
		}
		break;

	default:
		return 0;
		break;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
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
 * via DMA into specified buffer EOL_result, triggered by TIM3 interrupt.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (EOL_result[0] <= EOL_L_MIN)
	{
		carriageType = L;
		currentPosition = OFFSET_LEFT + 28;
		currentByte = 0;
	}
	else if (EOL_result[0] >= EOL_L_MAX)
	{
		carriageType = K;
		currentPosition = OFFSET_LEFT + 28;
		currentByte = 0;
	}
	else if (EOL_result[1] <= EOL_R_MIN)
	{
		carriageType = L;
		currentPosition = OFFSET_RIGHT - 28;
		currentByte = 25;
	}
	else if (EOL_result[1] >= EOL_R_MAX)
	{
		carriageType = K;
		currentPosition = OFFSET_RIGHT - 28;
		currentByte = 25;
	}
}

void testSolenoids(void)
{
	/* Just increment the bank, cover 0x00 to 0xFF */
	GPIOB->ODR = GPIOB->ODR + 1;
}

void testEncoders(void)
{
	sprintf(debugString, "POSITION: %i, DIRECTION: %i, BELT PHASE: %i\r\n", (uint8_t)TIM1->CNT, currentDir, getBP());
	CDC_Transmit_FS((uint8_t *)debugString, strlen(debugString));
}

void testEOL(void)
{
	sprintf(debugString, "IN0: %i, IN1: %i\r\n", EOL_result[0], EOL_result[1]);
	CDC_Transmit_FS((uint8_t *)debugString, strlen(debugString));
}

void calibrateEOL(void)
{

}

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

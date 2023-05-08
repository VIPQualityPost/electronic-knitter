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
#define FW_AYAB 0
#define SWAP_SOLENOID 1 /* If on rev1 boards with solenoid banks swapped. */

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
uint16_t calLeftMax;
uint16_t calLeftMin;
uint16_t calRightMax;
uint16_t calRightMin;

/* Machine variables */
uint8_t machineType;
uint8_t machineInitialized;
uint8_t machineHomed;
enum machineStates
{
	INIT,
	HOMING,
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
enum
{
	RIGHT,	  // 0, TIM1 increment
	LEFT	  // 1, TIM1 decrement
} currentDir; /* Using shift in currentPosition to determine direction. */

uint8_t currentPosition; /* Compare with TIM1->CNT to see relative movement. */
uint8_t bpShift;		 /* belt phase position during EOL stop. */

/* Pattern information */
uint16_t currentPattern; /* current solenoid pattern */
uint8_t *patternPointer;

uint8_t bitPattern[25];		/* The entire line to be knitted [25 bytes = 200 bits] 	*/
uint8_t startNeedle = 0xFF; /* Start needle for the pattern (subset of all needles) */
uint8_t stopNeedle = 0xFF;	/* End needle for the pattern (subset of all needles)	*/
uint8_t passStart;
uint8_t passStop;
uint8_t currentRow = 0; /* The row currently being worked by the machine. 		*/
uint8_t lastRow;		/* Flag if the line received from AYAB is the last line of the pattern */

/* General buffer*/

/* AYAB specific stuff*/
/* Firmware version information kept in ayab.h */
uint8_t *ayabBuf; /* Pointer to AYAB serial buffer. */
uint8_t continuousReport;
uint8_t crc8_cs; /* Checksum not used for anything now. */

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

	/* Setup machine */
	machineState = INIT;
	machineType = 0;
	carriageType = UNKNOWN;

	/* set the pointer to the start of the pattern */
	patternPointer = &bitPattern[0];

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
			if (FW_AYAB)
			{
				txAYAB(debug);
			}
			else
			{
				CDC_Transmit_FS((uint8_t *)debugString, strlen(debugString));
			}
			HAL_Delay(500);
			break;

		case HOMING:
			if (machineHomed)
			{
				machineState = RUN;
			}
			else if (currentPosition != TIM1->CNT)
			{
				currentPosition = TIM1->CNT;
				currentDir = (TIM1->CR1 | TIM_CR1_DIR) >> TIM_CR1_DIR_Pos;
				HAL_ADC_Start_DMA(&hadc1, (uint32_t *)EOL_result, 2);
			}
			break;

		case RUN:
			if (currentPosition != TIM1->CNT)
			{
				currentPosition = TIM1->CNT;
				currentDir = (TIM1->CR1 | TIM_CR1_DIR) >> TIM_CR1_DIR_Pos;
				// currentDir = TIM1->CNT - currentPosition;

				advanceCarriage();

				if (continuousReport && FW_AYAB)
				{
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
		/* Need to byteswap on rev1 boards because I swapped the banks on the connector.
		This is why we can't rely on reading GPIOB->ODR for the "true" current stage of solenoids. */
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

	/* See the note in HAL_ADC_ConvCpltCallback for information about setting the current
	position of the carriage. We have to modify this position based on the direction and
	type of carriage. */
	uint8_t realPosition = currentPosition - carriageOffset();
	uint8_t newByte;

	switch (currentDir)
	{
	case LEFT:
		/* Moving left */
		if (currentPosition <= carriageOffset())
		{
			/* If our position is at the edge of a solenoid bank then we can move to the next byte in bitPattern */
			if (currentPosition % 8 == 0)
			{
				if (patternPointer > &bitPattern[0] && patternPointer < &bitPattern[24])
				{
					patternPointer--;
					newByte = 1;
				}
			}

			if (carriageType == L)
			{
				realPosition -= 16;
			}
		}
		break;

	case RIGHT:
		/* Moving right */
		if (currentPosition >= carriageOffset())
		{
			if (currentPosition % 8 == 0)
			{
				if (patternPointer > &bitPattern[0] && patternPointer < &bitPattern[24] && 
							currentPosition > OFFSET_LEFT && currentPosition < OFFSET_RIGHT)
				{
					patternPointer++;
					newByte = 1;
				}
			}

			if (carriageType == L)
			{
				realPosition += 8;
			}
		}
		break;
	}

	/* Dig up the pixel pattern for the next two bytes of pixels. */
	/* patternPointer is uint8_t in order to work with bitPattern. We get
	the pattern by shifting the first byte up and then OR the lower byte
	(which is the pointer + 1). This is why we check if the pointer is at
	least zero and not above the 24th element in bitPattern earlier. We
	slide the window around by incrementing the pointer.*/

	/** [byte 0]		[byte 1]	....	[byte 25]
	 *	[patternPointer][patternPointer+1] ..........
	 *	012...		  8  9...		    15 16...  200
	 */

	uint16_t newPattern = (uint16_t)*patternPointer << 8 | *(patternPointer + 1);

	/* We need to swap upper and lower bytes if the line was started in
	a certain position. This is equivalent to adding 8 to the position
	of all 16 bits in the solenoid bank. */
	if (bpShift)
	{
		newPattern = (newPattern << 8) | (newPattern >> 8);
	}

	if (newByte)
	{
		writeSolenoids(newPattern);
	}

	/* Check if we have passed */
	if (realPosition == startNeedle)
	{
		passStart = 1;
	}
	else if (realPosition == stopNeedle)
	{
		passStop = 1;
	}

	if (passStart && passStop)
	{
		if (lastRow == 0)
		{
			/* We passed all the active needles so we can ask for the new line. */
			txAYAB(reqLine);

			/* Clear the progress flags. */
			passStart = 0;
			passStop = 0;
		}
		else
		{
			/* Turn off all the solenoids and do solid fill. */
			writeSolenoids(0x0000);
			machineInitialized = 0;
			machineState = INIT;
		}
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
	/* So these are actually backwards but because we only call this when
	we are interested in calculating the offset in the opposite direction.
	This might need a rework I think these values are determined experimentally. */
	switch (currentDir)
	{
	case RIGHT:
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

	case LEFT:
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
		// HAL_ADC_Start_DMA(&hadc1, (uint32_t *)EOL_result, 2);
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
	/* currentPosition is set at the center of carriage when it passes the EOL marker.
	But we need to set the solenoids so that the needles are selected during feed-in to
	the carriage. The offset between the sensor and the start of the bed is 12 stitches.
	The offset between the center of the carriage and the edge to the carriage is 28 stitches. */
	uint8_t beltPhase = getBP();

	switch (machineState)
	{
	case HOMING:
	case RUN:
		if (EOL_result[0] <= EOL_L_MIN)
		{
			/* L Carriage */
			carriageType = L;
			machineHomed = 1;
			TIM1->CNT = OFFSET_LEFT + 28;
			patternPointer = &bitPattern[0];
			/* From the table in service manual. */
			if (beltPhase)
			{
				bpShift = 0;
			}
			else
			{
				bpShift = 1;
			}
		}
		else if (EOL_result[0] >= EOL_L_MAX)
		{
			/* K Carriage */
			carriageType = K;
			machineHomed = 1;
			TIM1->CNT = OFFSET_LEFT + 28;
			patternPointer = &bitPattern[0];
			if (beltPhase)
			{
				bpShift = 1;
			}
			else
			{
				bpShift = 0;
			}
		}
		else if (EOL_result[1] <= EOL_R_MIN)
		{
			/* L Carriage */
			carriageType = L;
			machineHomed = 1;
			TIM1->CNT = OFFSET_RIGHT - 28;
			patternPointer = &bitPattern[24];
			if (beltPhase)
			{
				bpShift = 0;
			}
			else
			{
				bpShift = 1;
			}
		}
		else if (EOL_result[1] >= EOL_R_MAX)
		{
			/* K Carriage */
			carriageType = K;
			machineHomed = 1;
			TIM1->CNT = OFFSET_RIGHT - 28;
			patternPointer = &bitPattern[24];
			if (beltPhase)
			{
				bpShift = 0;
			}
			else
			{
				bpShift = 1;
			}
		}
		break;

	case CALIBRATE:
		if (EOL_result[0] < calLeftMin)
		{
			calLeftMin = EOL_result[0];
		}
		if (EOL_result[0] > calLeftMax)
		{
			calLeftMax = EOL_result[0];
		}
		if (EOL_result[1] < calRightMin)
		{
			calRightMin = EOL_result[1];
		}
		if (EOL_result[1] > calRightMax)
		{
			calRightMax = EOL_result[1];
		}

		sprintf(debugString, "LEFT: %i\t RIGHT: %i\nLEFT MAX: %i\t RIGHT MAX: %i\nLEFT MIN: %i\t RIGHT MIN: %i\n BELT PHASE: %i\r\n",
				EOL_result[0], EOL_result[1], calLeftMax, calRightMax, calLeftMin, calRightMin, beltPhase);

		if (FW_AYAB)
		{
			txAYAB(debug);
		}
		else
		{
			CDC_Transmit_FS((uint8_t *)debugString, strlen(debugString));
		}
		break;

	default:
		break;
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

/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

// includes
#include "stm32f1xx_hal.h"

void Error_Handler(void);

// defines
#define BUZZER_Pin GPIO_PIN_13
#define BUZZER_GPIO_Port GPIOC
#define EOL_L_Pin GPIO_PIN_0
#define EOL_L_GPIO_Port GPIOA
#define EOL_R_Pin GPIO_PIN_1
#define EOL_R_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_5
#define SPI_SCK_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define ENC_A_Pin GPIO_PIN_8
#define ENC_A_GPIO_Port GPIOA
#define ENC_B_Pin GPIO_PIN_9
#define ENC_B_GPIO_Port GPIOA
#define BELTPHASE_Pin GPIO_PIN_10
#define BELTPHASE_GPIO_Port GPIOA
#define HEARTBEAT_Pin GPIO_PIN_15
#define HEARTBEAT_GPIO_Port GPIOA

#ifdef __cplusplus
}
#endif

#endif
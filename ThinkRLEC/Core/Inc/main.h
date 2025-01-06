/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "L9963E.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern IWDG_HandleTypeDef hiwdg;
extern CAN_HandleTypeDef hcan;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern uint8_t can_broadcast[6][8];

extern L9963E_HandleTypeDef h9l;

extern uint8_t num_rlecs;
extern uint8_t rlec_ids[16];

struct module_info {
	uint16_t balcell;
	uint16_t vtot;
	uint16_t vcells[12];
	int8_t   vgpio[7];
	int8_t   tempchip;
	uint8_t  fault;
	uint8_t	 timeout;
};

extern struct module_info info[16];


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void flash_read(void);
void can_init (void);
void L9963E_utils_init(void);
void L9963E_utils_read_cells(uint8_t, uint8_t);
void L9963E_set_balance(uint8_t);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DIS_Pin GPIO_PIN_0
#define DIS_GPIO_Port GPIOA
#define BNE_Pin GPIO_PIN_2
#define BNE_GPIO_Port GPIOA
#define ISOFREQ_Pin GPIO_PIN_3
#define ISOFREQ_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define TXEN_Pin GPIO_PIN_1
#define TXEN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define DBG(...) \
            do { if (DEBUG) printf(__VA_ARGS__); } while (0)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

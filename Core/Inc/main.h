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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define TRUE 1
#define FALSE 0
#define DEBUGSLEEPFlag 1       // 0: check sleep mode ; 1: No check sleep mode. should be 0 for the production.
#define DOORDEBUG 0               //Should be 0 always.     0: get some fake information when not CAN available
#define FORCE_ANGLE 0          // 0: use sensor angle  1: force angle to a value
#define WATCHDOEENABLE  1   //1: use watchdog, 0: no watchdog
//#define DEBUGPRINT 1
#define NEWLATCH 1   //1: new latch is used
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define ENABLE_SENSOR_COMPENSASION 1

#define BKPSRAM_BASE 0x38800000
#define READ 0
#define WRITE 1

#define SWON 0
#define SWOFF 1
//#define MOTORSPEEDLOW 1  //0xA0
//#define MOTORSPEEDMID 2   //0xD0
//#define MOTORSPEEDHIGH 3   //0xFF

#define MAXINCLINATION 13.0   	 //max clication is 13 degree.
#define MININCLINATION -13.0		 //Min clication is -13 degree.
#define MAXTEMPERATURE 50.0   	 //max temperature is 50 degree.
#define MINTEMPERATURE -20.0		 //Min temperature is -20 degree.
#define INCLINATION_NORMAL_RANGE 10

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Braun_Op_Pin GPIO_PIN_2
#define Braun_Op_GPIO_Port GPIOE
#define Wakeup_Pin GPIO_PIN_3
#define Wakeup_GPIO_Port GPIOE
#define Fault_LED_Pin GPIO_PIN_4
#define Fault_LED_GPIO_Port GPIOE
#define CLUTCH_Pin GPIO_PIN_5
#define CLUTCH_GPIO_Port GPIOE
#define Latch_Pin GPIO_PIN_6
#define Latch_GPIO_Port GPIOE
#define CUR_SNS1_Pin GPIO_PIN_0
#define CUR_SNS1_GPIO_Port GPIOC
#define ERR_CAN_Pin GPIO_PIN_3
#define ERR_CAN_GPIO_Port GPIOC
#define CUR_SNS2_Pin GPIO_PIN_0
#define CUR_SNS2_GPIO_Port GPIOA
#define LATCH_DIR_Pin GPIO_PIN_1
#define LATCH_DIR_GPIO_Port GPIOA
#define BumpStripSignal_Pin GPIO_PIN_4
#define BumpStripSignal_GPIO_Port GPIOA
#define Red_LED_Pin GPIO_PIN_5
#define Red_LED_GPIO_Port GPIOA
#define White_LED_Pin GPIO_PIN_6
#define White_LED_GPIO_Port GPIOA
#define Yellow_LED_Pin GPIO_PIN_7
#define Yellow_LED_GPIO_Port GPIOA
#define Door_Encode_Pin GPIO_PIN_4
#define Door_Encode_GPIO_Port GPIOC
#define LATCH_Disable_Pin GPIO_PIN_5
#define LATCH_Disable_GPIO_Port GPIOC
#define Door_Open_Pin GPIO_PIN_0
#define Door_Open_GPIO_Port GPIOB
#define Door_Open_EXTI_IRQn EXTI0_IRQn
#define White_LED_12V_Pin GPIO_PIN_1
#define White_LED_12V_GPIO_Port GPIOB
#define Kneel_Disable_Pin GPIO_PIN_2
#define Kneel_Disable_GPIO_Port GPIOB
#define Lock_Status_Pin GPIO_PIN_7
#define Lock_Status_GPIO_Port GPIOE
#define Spare2_Pin GPIO_PIN_8
#define Spare2_GPIO_Port GPIOE
#define Conversion_Dis_Pin GPIO_PIN_9
#define Conversion_Dis_GPIO_Port GPIOE
#define Outside_Handle_Pin GPIO_PIN_11
#define Outside_Handle_GPIO_Port GPIOE
#define Outside_Handle_EXTI_IRQn EXTI15_10_IRQn
#define Door_Ajar_Pin GPIO_PIN_13
#define Door_Ajar_GPIO_Port GPIOE
#define Door_Ajar_EXTI_IRQn EXTI15_10_IRQn
#define LATCH_VSO_Pin GPIO_PIN_10
#define LATCH_VSO_GPIO_Port GPIOB
#define LATCH_PWM_Pin GPIO_PIN_11
#define LATCH_PWM_GPIO_Port GPIOD
#define Ramp_Light_Pin GPIO_PIN_15
#define Ramp_Light_GPIO_Port GPIOD
#define ERR_1_Pin GPIO_PIN_7
#define ERR_1_GPIO_Port GPIOC
#define EN_1_Pin GPIO_PIN_8
#define EN_1_GPIO_Port GPIOC
#define STB_1_Pin GPIO_PIN_9
#define STB_1_GPIO_Port GPIOC
#define Lock_Release_Enable_Pin GPIO_PIN_0
#define Lock_Release_Enable_GPIO_Port GPIOD
#define Lock_Release_Trigger_Pin GPIO_PIN_1
#define Lock_Release_Trigger_GPIO_Port GPIOD
#define Lock_Unlock_Coil_Pin GPIO_PIN_2
#define Lock_Unlock_Coil_GPIO_Port GPIOD
#define RS_EN_Pin GPIO_PIN_5
#define RS_EN_GPIO_Port GPIOB
#define RD_EN_Pin GPIO_PIN_6
#define RD_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define WAKE_DOOR_COUNT_SETTING 100
#define OVERCURRENTTHRESHOLD   10    //10A?
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

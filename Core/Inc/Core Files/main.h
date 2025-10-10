/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Common includes/defines & externs for main.c and modules
  ******************************************************************************
  * Notes:
  *  - Exposes TIM handle externs and tick flags produced by irq_handler_callback.c
  *  - Maps GPIO pins for LEDs, buttons, fans, PWM channels
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Temper_NTC_1_Pin GPIO_PIN_0
#define Temper_NTC_1_GPIO_Port GPIOA
#define Temper_NTC_2_Pin GPIO_PIN_2
#define Temper_NTC_2_GPIO_Port GPIOA
#define Temper_NTC_3_Pin GPIO_PIN_3
#define Temper_NTC_3_GPIO_Port GPIOA
#define RING_LED_PWM_Pin GPIO_PIN_1
#define RING_LED_PWM_GPIO_Port GPIOA

#define MOTOR_CURRENT_Pin GPIO_PIN_4
#define MOTOR_CURRENT_Pin_Port GPIOA
#define LED2_PC_Pin GPIO_PIN_5
#define LED2_PC_GPIO_Port GPIOA
#define MOTOR_FG_T3_CH1_Pin GPIO_PIN_6
#define MOTOR_FG_T3_CH1_GPIO_Port GPIOA

#define MOTOR_ROT_PA7_Pin GPIO_PIN_7
#define MOTOR_ROT_PA7_GPIO_Port GPIOA
#define MOTO_PWM_T3_CH3_Pin GPIO_PIN_0
#define MOTO_PWM_T3_CH3_GPIO_Port GPIOB
#define MOTOR_BRAK_PB1_Pin GPIO_PIN_1
#define MOTOR_BRAK_PB1_GPIO_Port GPIOB
#define FREEZ_BTN_EXT10_Pin GPIO_PIN_10
#define FREEZ_BTN_EXT10_GPIO_Port GPIOB
#define FREEZ_BTN_EXT10_EXTI_IRQn EXTI15_10_IRQn
#define LIGHT_BTN_EXT11_Pin GPIO_PIN_11
#define LIGHT_BTN_EXT11_GPIO_Port GPIOB
#define LIGHT_BTN_EXT11_EXTI_IRQn EXTI15_10_IRQn
#define LED_BLUE_Pin GPIO_PIN_12
#define LED_BLUE_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_13
#define LED_GREEN_GPIO_Port GPIOB
#define LED_WHITE_Pin GPIO_PIN_14
#define LED_WHITE_GPIO_Port GPIOB
#define LED3_PC_Pin GPIO_PIN_15
#define LED3_PC_GPIO_Port GPIOB
#define INVERTER_PWM_Pin GPIO_PIN_8
#define INVERTER_PWM_GPIO_Port GPIOA
#define PWOER_CTRL_PC1_Pin GPIO_PIN_11
#define PWOER_CTRL_PC1_GPIO_Port GPIOA
#define PC2_DET_EXT12_Pin GPIO_PIN_12
#define PC2_DET_EXT12_GPIO_Port GPIOA
#define PC2_DET_EXT12_EXTI_IRQn EXTI15_10_IRQn
#define LED_R_Pin GPIO_PIN_15
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_3
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_4
#define LED_B_GPIO_Port GPIOB
#define LED_W_Pin GPIO_PIN_5
#define LED_W_GPIO_Port GPIOB
#define FAN_PWM_TIM4CH1_Pin GPIO_PIN_6
#define FAN_PWM_TIM4CH1_GPIO_Port GPIOB
#define FAN_FG_TIM4CH2_Pin GPIO_PIN_7
#define FAN_FG_TIM4CH2_GPIO_Port GPIOB
#define FAN2_PWM_TIM4CH3_Pin GPIO_PIN_8
#define FAN2_PWM_TIM4CH3_GPIO_Port GPIOB
#define FAN2_FG_TIM4CH4_Pin GPIO_PIN_9
#define FAN2_FG_TIM4CH4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Peripheral handle externs (defined in main.c) */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Minimal, optimized tick flags produced by irq_handler_callback.c */

#define FAN2_FG_CHANNEL   TIM_CHANNEL_4
#define FAN2_PWM_CHANNEL   TIM_CHANNEL_3
#define FAN_FG_CHANNEL   TIM_CHANNEL_2
#define FAN_PWM_CHANNEL   TIM_CHANNEL_1
#define FAN2_FG_TIM       htim4
#define FAN_FG_TIM       htim4
#define FAN_PWM_TIM      htim4
#define MOTOR_FG_CHANNEL  TIM_CHANNEL_1
#define MOTOR_FG_TIM      htim3
#define MOTOR_PWM_TIM     htim3
#define MOTOR_PWM_CHANNEL   TIM_CHANNEL_3

#define INVERTER_PWM_TIM   htim1
#define INVERTER_PWM_CHANNEL TIM_CHANNEL_1

#define RING_LED_PWM_TIM  htim2
#define RING_LED_PWM_CHANNEL  TIM_CHANNEL_2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

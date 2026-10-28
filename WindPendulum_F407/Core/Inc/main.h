#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOD
#define MOTOR_ENABLE_Pin GPIO_PIN_7
#define MOTOR_ENABLE_GPIO_Port GPIOD
#define LASER_Pin GPIO_PIN_8
#define LASER_GPIO_Port GPIOD
#define MOTOR_FAULT_Pin GPIO_PIN_10
#define MOTOR_FAULT_GPIO_Port GPIOD
#define IMU_INT1_Pin GPIO_PIN_0
#define IMU_INT1_GPIO_Port GPIOD
#define IMU_INT2_Pin GPIO_PIN_1
#define IMU_INT2_GPIO_Port GPIOD
#define IMU_CS_Pin GPIO_PIN_7
#define IMU_CS_GPIO_Port GPIOE

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif

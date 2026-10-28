#ifndef STM32F4xx_HAL_CONF_H
#define STM32F4xx_HAL_CONF_H

#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_SPI_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

#define HSE_VALUE              8000000U
#define HSE_STARTUP_TIMEOUT    100U
#define HSI_VALUE              16000000U
#define LSE_VALUE              32768U
#define LSE_STARTUP_TIMEOUT   5000U
#define LSI_VALUE              32000U
#define EXTERNAL_CLOCK_VALUE   12288000U
#define VDD_VALUE              3300U
#define TICK_INT_PRIORITY      15U
#define USE_RTOS               0U
#define PREFETCH_ENABLE        1U
#define INSTRUCTION_CACHE_ENABLE 1U
#define DATA_CACHE_ENABLE      1U

#define USE_HAL_ADC_REGISTER_CALLBACKS  0U
#define USE_HAL_I2C_REGISTER_CALLBACKS  0U
#define USE_HAL_SPI_REGISTER_CALLBACKS  0U
#define USE_HAL_TIM_REGISTER_CALLBACKS  0U
#define USE_HAL_UART_REGISTER_CALLBACKS 0U

#define assert_param(expr) ((void)0U)

#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32f4xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32f4xx_hal_gpio.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f4xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32f4xx_hal_cortex.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32f4xx_hal_dma.h"
#endif
#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED
#include "stm32f4xx_hal_i2c.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_pwr_ex.h"
#endif
#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32f4xx_hal_spi.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_usart.h"
#endif

#endif

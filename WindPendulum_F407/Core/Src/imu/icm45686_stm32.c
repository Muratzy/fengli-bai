#include "icm45686_stm32.h"

#include "main.h"
#include "imu/inv_imu_driver.h"

#include <string.h>

#define ICM45686_EXPECTED_WHOAMI 0xE9U
#define ICM45686_SPI_TIMEOUT_MS  10U
#define ICM45686_SPI_MAX_READ   32U

static inv_imu_device_t g_imu_device;

static void ICM45686_DelayUs(uint32_t microseconds)
{
  const uint32_t start = DWT->CYCCNT;
  const uint32_t cycles = (SystemCoreClock / 1000000U) * microseconds;
  while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
  }
}

static int ICM45686_SpiRead(void *context, uint8_t reg, uint8_t *data,
                            uint32_t length)
{
  SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)context;
  uint8_t address = (uint8_t)(reg | 0x80U);
  uint8_t dummy[ICM45686_SPI_MAX_READ];
  HAL_StatusTypeDef status;

  if (spi == NULL || data == NULL || length == 0U ||
      length > ICM45686_SPI_MAX_READ) {
    return -1;
  }

  memset(dummy, 0xFF, length);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(spi, &address, 1U, ICM45686_SPI_TIMEOUT_MS);
  if (status == HAL_OK) {
    status = HAL_SPI_TransmitReceive(spi, dummy, data, (uint16_t)length,
                                     ICM45686_SPI_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return (status == HAL_OK) ? 0 : -1;
}

static int ICM45686_SpiWrite(void *context, uint8_t reg,
                             const uint8_t *data, uint32_t length)
{
  SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)context;
  HAL_StatusTypeDef status;

  if (spi == NULL || data == NULL || length == 0U || length > UINT16_MAX) {
    return -1;
  }

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(spi, &reg, 1U, ICM45686_SPI_TIMEOUT_MS);
  if (status == HAL_OK) {
    status = HAL_SPI_Transmit(spi, (uint8_t *)data, (uint16_t)length,
                              ICM45686_SPI_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return (status == HAL_OK) ? 0 : -1;
}

int ICM45686_STM32_Init(void)
{
  uint8_t who_am_i = 0U;

  memset(&g_imu_device, 0, sizeof(g_imu_device));
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  g_imu_device.transport.context = &hspi2;
  g_imu_device.transport.read_reg = ICM45686_SpiRead;
  g_imu_device.transport.write_reg = ICM45686_SpiWrite;
  g_imu_device.transport.serif_type = UI_SPI4;
  g_imu_device.transport.sleep_us = ICM45686_DelayUs;

  ICM45686_DelayUs(3000U);
  if (inv_imu_get_who_am_i(&g_imu_device, &who_am_i) != INV_IMU_OK ||
      who_am_i != ICM45686_EXPECTED_WHOAMI) {
    return -1;
  }
  if (inv_imu_soft_reset(&g_imu_device) != INV_IMU_OK ||
      inv_imu_get_who_am_i(&g_imu_device, &who_am_i) != INV_IMU_OK ||
      who_am_i != ICM45686_EXPECTED_WHOAMI) {
    return -1;
  }

  if (inv_imu_set_accel_fsr(&g_imu_device,
                            ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G) != INV_IMU_OK ||
      inv_imu_set_accel_frequency(&g_imu_device,
                                  ACCEL_CONFIG0_ACCEL_ODR_800_HZ) != INV_IMU_OK ||
      inv_imu_set_gyro_fsr(&g_imu_device,
                           GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS) != INV_IMU_OK ||
      inv_imu_set_gyro_frequency(&g_imu_device,
                                 GYRO_CONFIG0_GYRO_ODR_800_HZ) != INV_IMU_OK ||
      inv_imu_set_accel_mode(&g_imu_device, PWR_MGMT0_ACCEL_MODE_LN) != INV_IMU_OK ||
      inv_imu_set_gyro_mode(&g_imu_device, PWR_MGMT0_GYRO_MODE_LN) != INV_IMU_OK) {
    return -1;
  }

  return 0;
}

int ICM45686_STM32_Read(ImuSample *sample)
{
  inv_imu_sensor_data_t sensor_data;

  if (sample == NULL ||
      inv_imu_get_register_data(&g_imu_device, &sensor_data) != INV_IMU_OK) {
    return -1;
  }

  for (uint32_t axis = 0U; axis < 3U; ++axis) {
    sample->accel_raw[axis] = sensor_data.accel_data[axis];
    sample->gyro_raw[axis] = sensor_data.gyro_data[axis];
    sample->accel_g[axis] = (float)sensor_data.accel_data[axis] *
                            (16.0F / 32768.0F);
    sample->gyro_dps[axis] = (float)sensor_data.gyro_data[axis] *
                             (2000.0F / 32768.0F);
  }
  sample->temperature_c = 25.0F + (float)sensor_data.temp_data / 128.0F;
  return 0;
}

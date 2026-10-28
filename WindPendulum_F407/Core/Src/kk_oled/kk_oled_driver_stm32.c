#include "main.h"
#include "kk_oled_driver.h"
#include "kk_oled_internal.h"

#define OLED_I2C_ADDRESS       0x78U
#define OLED_CONTROL_COMMAND   0x00U
#define OLED_CONTROL_DATA      0x40U
#define OLED_I2C_TIMEOUT_MS    100U

static OLED_Status OLED_Send(uint8_t control, const uint8_t *data,
                             uint16_t length)
{
  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Write(&hi2c1, OLED_I2C_ADDRESS, control,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, length,
                             OLED_I2C_TIMEOUT_MS);
  return (status == HAL_OK) ? OLED_OK : OLED_ERROR;
}

static OLED_Status OLED_SendCommands(const uint8_t *commands,
                                     uint16_t length)
{
  return OLED_Send(OLED_CONTROL_COMMAND, commands, length);
}

static OLED_Status OLED_SendPage(uint8_t page, uint8_t first_column,
                                 const uint8_t *pixels, uint16_t length)
{
  uint8_t address[3] = {
      (uint8_t)(0xB0U | page),
      (uint8_t)(first_column & 0x0FU),
      (uint8_t)(0x10U | (first_column >> 4U))
  };

  if (OLED_SendCommands(address, sizeof(address)) != OLED_OK) {
    return OLED_ERROR;
  }
  return OLED_Send(OLED_CONTROL_DATA, pixels, length);
}

OLED_Status OLED_DriverInit(void)
{
  static const uint8_t init_commands[] = {
      0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
      0x8DU, 0x14U, 0x20U, 0x02U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
      0x81U, 0x7FU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U,
      0x2EU, 0xAFU
  };
  static const uint8_t zeros[OLED_PHYSICAL_WIDTH] = {0};

  HAL_Delay(30U);
  if (OLED_SendCommands(init_commands, sizeof(init_commands)) != OLED_OK) {
    return OLED_ERROR;
  }
  for (uint8_t page = 0U; page < OLED_PHYSICAL_PAGES; ++page) {
    if (OLED_SendPage(page, 0U, zeros, sizeof(zeros)) != OLED_OK) {
      return OLED_ERROR;
    }
  }
  {
    const uint8_t display_on = 0xAFU;
    return OLED_SendCommands(&display_on, sizeof(display_on));
  }
}

OLED_Status OLED_DriverWriteBlocking(void)
{
  const uint8_t *buffer = OLED_InternalGetTransferBuffer();

  for (uint8_t page = 0U; page < OLED_PHYSICAL_PAGES; ++page) {
    const uint8_t first = OLED_InternalGetTransferMinX(page);
    const uint8_t last = OLED_InternalGetTransferMaxX(page);
    if (first >= OLED_PHYSICAL_WIDTH) {
      continue;
    }
    if (OLED_SendPage(page, first,
                      buffer + (uint16_t)page * OLED_PHYSICAL_WIDTH + first,
                      (uint16_t)last - first + 1U) != OLED_OK) {
      return OLED_ERROR;
    }
  }
  return OLED_OK;
}

OLED_Status OLED_DriverWriteIT(void)
{
  return OLED_ERROR;
}

OLED_Status OLED_DriverWriteDMA(void)
{
  return OLED_ERROR;
}

bool OLED_DriverIsBusy(void)
{
  return false;
}

OLED_Status OLED_DriverSetContrast(uint8_t value)
{
  const uint8_t commands[] = {0x81U, value};
  return OLED_SendCommands(commands, sizeof(commands));
}

OLED_Status OLED_DriverSetPowerSave(bool enable)
{
  const uint8_t command = enable ? 0xAEU : 0xAFU;
  return OLED_SendCommands(&command, sizeof(command));
}

void OLED_DriverHandleMemTxComplete(void)
{
}

void OLED_DriverHandleError(void)
{
}

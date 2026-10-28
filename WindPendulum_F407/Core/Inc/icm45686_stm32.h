#ifndef ICM45686_STM32_H
#define ICM45686_STM32_H

#include "pendulum_app.h"

#include <stdint.h>

int ICM45686_STM32_Init(void);
int ICM45686_STM32_Read(ImuSample *sample);

#endif

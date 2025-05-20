#pragma once

#include "stm32f4xx_hal.h"

#include <stdbool.h>




int sAPP_CLI_Init();

void sAPP_CLI_RecvTask(void* param);
void sAPP_CLI_SendTask(void* param);

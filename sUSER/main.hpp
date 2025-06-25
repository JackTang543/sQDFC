#pragma once

#include "math.h"
#include "stm32f4xx_hal.h"



#include "sAPP_Debug.h"
#include "sAPP_AHRS.hpp"
#include "sAPP_Tasks.hpp"
#include "sAPP_CLI.hpp"
#include "sAPP_ParamSave.hpp"

#include "sDRV_ICM45686.h"
#include "sDRV_LIS3MDLTR.h"
#include "sDRV_SPL06.h"
#include "sDRV_MB85RCxx.h"
#include "SPL06001.h"
#include "sDRV_DSHOT.hpp"


#include "sBSP_GPIO.h"
#include "sBSP_SPI.h"
#include "sBSP_UART.h"
#include "sBSP_RCC.h"
#include "sBSP_TIM.h"
#include "sBSP_DMA.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "cm_backtrace.h"

#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cstdio>



#include "ekf_AltEst6\ekf_AltEst6.h"







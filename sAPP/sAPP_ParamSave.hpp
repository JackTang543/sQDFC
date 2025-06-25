#pragma once


#include "main.hpp"

#include "sAPP_AHRS.hpp"


#include "defines.h"



#include "sDRV_MB85RCxx.h"




void sAPP_ParamSave_Init();


int sAPP_ParamSave_SaveGyrSBias(float x_bias,float y_bias,float z_bias,float temp);
int sAPP_ParamSave_ReadGyrSBias(float* x_bias,float* y_bias,float* z_bias,float* temp);
int sAPP_ParamSave_SaveAccSBias(float x_bias,float y_bias,float z_bias,float temp);
int sAPP_ParamSave_ReadAccSBias(float* x_bias,float* y_bias,float* z_bias,float* temp);

void sAPP_ParamSave_CaliIMU();











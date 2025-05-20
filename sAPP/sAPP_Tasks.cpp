#include "sAPP_Tasks.hpp"

#include "main.hpp"




void sAPP_Tasks_Devices(void* param){
    for(;;){
        //处理按键
        // sGBD_Handler();
        //处理二值化设备
        // BinOutDrv.update();
        


        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}


// #include "sDRV_AHT20.h"

void sAPP_Tasks_500ms(void* param){
    for(;;){
        HAL_GPIO_TogglePin(RUN_LED_GPIO_Port, RUN_LED_Pin); // 翻转RUN LED引脚


        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void sAPP_Tasks_1000ms(void* param){
    for(;;){

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sAPP_Tasks_TaskMang(void* param){

    for(;;){
        sAPP_Tasks_PrintTaskMang();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sAPP_Tasks_LoopTask(void* param){
    for(;;){
        // loop();
    }
}

void sAPP_AHRS_Periodic(void* param){
    for(;;){
        ahrs.periodicProcess();
        //Freq=50Hz
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// void sAPP_Tasks_FormatFeRAM(void* param){
//     menu.setLock("FeRAM","FeRAM is \nformatting...");
//     sBSP_UART_Debug_Printf("FeRAM开始格式化!\n");
//     if(sDRV_MB85RCxx_Format(0) == 0){
//         sBSP_UART_Debug_Printf("FeRAM格式化完成!\n");
//     }else{
//         sBSP_UART_Debug_Printf("FeRAM格式化失败!\n");
//     }
//     menu.setUnlock();
//     vTaskDelete(NULL);
// }

// static void calibrateIMU(void* param){
//     menu.setLock("IMU","IMU is \ncalibrating...");
//     sAPP_ParamSave_CaliIMU();




//     menu.setUnlock();
//     vTaskDelete(NULL);
// }


// static void imu_calib_acc_bias(void* param){
//     menu.setLock("IMU Calib","Accel is \ncalibrating...");
//     log_info("在1s后开始校准加速度计静态零偏...");
//     vTaskDelay(1000);

//     //校准加速度计静态零偏
//     AHRS::IMU_StaticBias imu_sbias;
//     //静态零偏采样点数
//     const uint16_t sample_points = 2000;
//     ahrs.calcBias(sample_points,imu_sbias);
//     float temp = ahrs.raw_data.imu_temp;

//     log_info("测量点数:%u,加速度计静态零偏:%.3f,%.3f,%.3f",sample_points,imu_sbias.acc_x,imu_sbias.acc_y,imu_sbias.acc_z);

//     //静态零偏阈值,超过说明没有把系统放平
//     const float bias_limit = 0.3f;
//     if(fabs(imu_sbias.acc_x) > bias_limit || fabs(imu_sbias.acc_y) > bias_limit || fabs(imu_sbias.acc_z) > bias_limit){
//         log_error("加速度计静态零偏数据超过阈值,请将主板放平后重试! 本次校准无效");
//         menu.setUnlock();
//         //提示用户
//         menu.createTipsBox("Acc Calib FAIL","Plase take the\nsystem on\nflat surface!");
//         goto END;
//     }

//     //保存静态零偏数据到FeRAM
//     if(sAPP_ParamSave_SaveAccSBias(imu_sbias.acc_x,imu_sbias.acc_y,imu_sbias.acc_z,temp) != 0){
//         log_error("加速度计静态零偏数据保存失败!");
//         //提示用户
//         menu.createTipsBox("Acc Calib FAIL","Static bias\nsave fail!");
//         goto END;
//     }

//     //保存静态零偏数据到AHRS
//     ahrs.updateAccSBias(imu_sbias.acc_x,imu_sbias.acc_y,imu_sbias.acc_z);

//     log_info("加速度计静态零偏校准完成,保存成功 OK");
//     menu.createTipsBox("Acc Calib OK","Static bias\nsave success!");


// END:
//     menu.setUnlock();
//     vTaskDelete(NULL);
// }

// static void imu_calib_gyr_bias(void* param){
//     menu.setLock("IMU Calib","Gyro is \ncalibrating...");
//     log_info("在1s后开始校准陀螺仪静态零偏...");
//     vTaskDelay(1000);

//     //校准陀螺仪静态零偏
//     AHRS::IMU_StaticBias imu_sbias;
//     //静态零偏采样点数
//     const uint16_t sample_points = 2000;
//     float temp = ahrs.raw_data.imu_temp;
//     ahrs.calcBias(sample_points,imu_sbias);

//     log_info("测量点数:%u,陀螺仪静态零偏:%.3f,%.3f,%.3f",sample_points,imu_sbias.gyr_x,imu_sbias.gyr_y,imu_sbias.gyr_z);

//     //静态零偏阈值,超过说明系统没有在静止状态或者陀螺仪有问题
//     const float bias_limit = ahrs.getIMUType() == AHRS::IMUType::ICM45686 ? 0.3f : 0.5f;
//     if(fabs(imu_sbias.gyr_x) > bias_limit || fabs(imu_sbias.gyr_y) > bias_limit || fabs(imu_sbias.gyr_z) > bias_limit){
//         log_error("陀螺仪静态零偏数据超过阈值,可能是系统未处于静止状态或者陀螺仪性能不佳 本次校准无效");
//         menu.createTipsBox("Gyro Calib FAIL","Plase keep the\nsystem in a\nstatic state!");
        
//         goto END;
//     }

//     //保存静态零偏数据到FeRAM
//     if(sAPP_ParamSave_SaveGyrSBias(imu_sbias.gyr_x,imu_sbias.gyr_y,imu_sbias.gyr_z,temp) != 0){
//         log_error("陀螺仪静态零偏数据保存失败!");
//         menu.createTipsBox("Gyro Calib FAIL","Static bias\nsave fail!");
//         goto END;
//     }

//     //保存静态零偏数据到AHRS
//     ahrs.updateGyrSBias(imu_sbias.gyr_x,imu_sbias.gyr_y,imu_sbias.gyr_z);

//     log_info("陀螺仪静态零偏校准完成,保存成功 OK");
//     menu.createTipsBox("Gyro Calib OK","Static bias\nsave success!");

// END:
//     menu.setUnlock();
//     vTaskDelete(NULL);
// }


// void sAPP_Tasks_ReadIMUCaliVal(){
//     AHRS::IMU_StaticBias imu_sbias;
//     float acc_bias_temp = 0.0f;
//     float gyr_bias_temp = 0.0f;
//     bool is_acc_bias_valid = false;
//     bool is_gyr_bias_valid = false;
//     if(sAPP_ParamSave_ReadAccSBias(&imu_sbias.acc_x,&imu_sbias.acc_y,&imu_sbias.acc_z,&acc_bias_temp) != 0){
//         log_error("加速度计静态零偏数据无效!");
//         is_acc_bias_valid = false;
//     }else{
//         //检查校准时的温度和当前温度差
//         ahrs.getIMUData();
//         float diff_temp = fabs(acc_bias_temp - ahrs.raw_data.imu_temp);
//         if(diff_temp > 3.0f){
//             log_warn("加速度计静态零偏数据温度差过大,建议重新校准,偏差:%.1f摄氏度",diff_temp);
//             is_acc_bias_valid = false;
//         }
//         else{
//             is_acc_bias_valid = true;
//         }
//     }

//     if(sAPP_ParamSave_ReadGyrSBias(&imu_sbias.gyr_x,&imu_sbias.gyr_y,&imu_sbias.gyr_z,&gyr_bias_temp) != 0){
//         log_error("陀螺仪静态零偏数据无效!");
//         is_gyr_bias_valid = false;
//     }else{
//         //检查校准时的温度和当前温度差
//         ahrs.getIMUData();
//         float diff_temp = fabs(gyr_bias_temp - ahrs.raw_data.imu_temp);
//         if(diff_temp > 3.0f){
//             log_warn("陀螺仪静态零偏数据温度差过大,建议重新校准,偏差:%.1f摄氏度",diff_temp);
//             is_gyr_bias_valid = false;
//         }
//         else{
//             is_gyr_bias_valid = true;
//         }
//         is_gyr_bias_valid = true;
//     }

//     if(is_acc_bias_valid == true){
//         ahrs.updateAccSBias(imu_sbias.acc_x,imu_sbias.acc_y,imu_sbias.acc_z);
//         log_info("加速度计静态零偏数据已读取:%.4f,%.4f,%.4f",imu_sbias.acc_x,imu_sbias.acc_y,imu_sbias.acc_z);
//         ahrs.setIMUState(AHRS::IMUState::OK);
//     }else{
//         ahrs.setIMUState(AHRS::IMUState::NEED_CALIB);
//     }

//     if(is_gyr_bias_valid == true){
//         ahrs.updateGyrSBias(imu_sbias.gyr_x,imu_sbias.gyr_y,imu_sbias.gyr_z);
//         log_info("陀螺仪静态零偏数据已读取:%.4f,%.4f,%.4f",imu_sbias.gyr_x,imu_sbias.gyr_y,imu_sbias.gyr_z);
//         ahrs.setIMUState(AHRS::IMUState::OK);
//     }else{
//         ahrs.setIMUState(AHRS::IMUState::NEED_CALIB);
//     }
// }

// void sAPP_Tasks_ProtectTask(void* param){
//     bool is_low_bat = false;
//     bool is_first_tip = true;

//     vTaskDelay(5000);
//     for(;;){
//         if(xSemaphoreTake(car.mutex,200) == pdTRUE){
//             //电池电压过低保护
//             if(car.batt_volt < 10.5f){
//                 is_low_bat = true;
//                 // sBSP_UART_Debug_Printf("[ERR ]电池电压过低:%.2fv\n",car.batt_volt);
//             }
//             //回差500mV,防止重复触发
//             else if(car.batt_volt > 11.0f){
//                 is_low_bat = false;
//                 is_first_tip = true;
//             }
//             xSemaphoreGive(car.mutex);
//         }

//         if(is_low_bat && is_first_tip){
//             is_first_tip = false;
//             menu.createTipsBox("LOW BATT WARN","Battery voltage\nis too low!");

//         }

//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     }
// }




void sAPP_Tasks_CreateAll(){
    //控制算法
    // xTaskCreate(sAPP_BlcCtrl_CtrlTask   , "CtrlTask"     , 4096 / sizeof(int), NULL, 4, NULL);

    //姿态估计算法
    xTaskCreate(sAPP_AHRS_Task          , "AHRS"         , 16384 / sizeof(int), NULL, 4, NULL);
    //AHRS的杂项任务
    xTaskCreate(sAPP_AHRS_Periodic      , "AHRS_Period"  , 4096 / sizeof(int), NULL, 3, NULL);

    //OLED刷屏 20Hz  
    // xTaskCreate(sAPP_Tasks_OLEDHdr      , "OLED"         , 16384 / sizeof(int), NULL, 2, NULL);
    xTaskCreate(sAPP_Tasks_Devices      , "Devices"      , 1024 / sizeof(int), NULL, 1, NULL);

    xTaskCreate(sAPP_CLI_RecvTask           , "CLI Recv"     , 2048 / sizeof(int), NULL, 1, NULL);
    xTaskCreate(sAPP_CLI_SendTask           , "CLI Send"     , 2048 / sizeof(int), NULL, 1, NULL);

    // xTaskCreate(sAPP_Car_InfoUpdateTask , "CarInfoUp"    , 2048 / sizeof(int), NULL, 1, NULL);
    // xTaskCreate(sAPP_Tasks_ProtectTask  , "Protect"      , 2048 / sizeof(int), NULL, 1, NULL);
    xTaskCreate(sAPP_Tasks_500ms        , "500ms"        , 1024 / sizeof(int), NULL, 1, NULL);
    // xTaskCreate(sAPP_Tasks_1000ms       , "1000ms"       , 1024 / sizeof(int), NULL, 1, NULL);
    // xTaskCreate(sAPP_Tasks_TaskMang     , "TaskMang"     , 2048 / sizeof(int), NULL, 1, NULL);
    // xTaskCreate(sAPP_Tasks_LoopTask     , "Loop"         , 8192 / sizeof(int), NULL, 2, NULL);

}


// void sAPP_Tasks_StartFormatFeRAM(){
//     xTaskCreate(sAPP_Tasks_FormatFeRAM  , "ForatFeRAM"   ,  512 / sizeof(int), NULL, 1, NULL);
// }

// void sAPP_Tasks_StartCalibrateIMU(){
//     xTaskCreate(calibrateIMU            , "calibrateIMU" , 1024 / sizeof(int), NULL, 1, NULL);
// }

// void sAPP_Tasks_StartCalibAccBias(){
//     xTaskCreate(imu_calib_acc_bias      , "calibAccBias" , 1024 / sizeof(int), NULL, 2, NULL);
// }

// void sAPP_Tasks_StartCalibGyrBias(){
//     xTaskCreate(imu_calib_gyr_bias      , "calibGyrBias" , 1024 / sizeof(int), NULL, 2, NULL);
// }


void sAPP_Tasks_PrintTaskMang(){
    const char task_state[]={'r','R','B','S','D'};
    volatile UBaseType_t uxArraySize, x;
    unsigned portSHORT ulTotalRunTime,ulStatsAsPercentage;
    /* 获取任务总数目 */
    uxArraySize = uxTaskGetNumberOfTasks();
    sBSP_UART_Debug_Printf("当前任务数量:%d\n",uxArraySize);
    TaskStatus_t pxTaskStatusArray[16];
    /*获取每个任务的状态信息 */
    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray,uxArraySize,&ulTotalRunTime);
    sBSP_UART_Debug_Printf("任务名        状态       ID       优先级       堆栈剩余      CPU使用率\n");
    /* 避免除零错误 */
    if( ulTotalRunTime > 0 ){
        /* 将获得的每一个任务状态信息部分的转化为程序员容易识别的字符串格式 */
        for(x = 0; x < uxArraySize; x++){
            char tmp[128];
            /* 计算任务运行时间与总运行时间的百分比。*/
            ulStatsAsPercentage =(uint16_t)(pxTaskStatusArray[x].ulRunTimeCounter)*100 / ulTotalRunTime;
 
            if( ulStatsAsPercentage > 0UL ){
               sprintf(tmp,"%-15s%-10c%-10lu%-12lu%-12d%d%%",pxTaskStatusArray[ x].pcTaskName,task_state[pxTaskStatusArray[ x ].eCurrentState],
                                                             pxTaskStatusArray[ x ].xTaskNumber,pxTaskStatusArray[ x].uxCurrentPriority,
                                                             pxTaskStatusArray[ x ].usStackHighWaterMark,ulStatsAsPercentage);
            }
            else{
                /* 任务运行时间不足总运行时间的1%*/
                sprintf(tmp,"%-15s%-10c%-10lu%-12lu%-12dt<1%%",pxTaskStatusArray[x ].pcTaskName,task_state[pxTaskStatusArray[ x ].eCurrentState],
                                                               pxTaskStatusArray[ x ].xTaskNumber,pxTaskStatusArray[ x].uxCurrentPriority,
                                                               pxTaskStatusArray[ x ].usStackHighWaterMark);               
            }
           sBSP_UART_Debug_Printf("%s\n",tmp);
        }
    }
    sBSP_UART_Debug_Printf("任务状态:   r-运行  R-就绪  B-阻塞  S-挂起  D-删除\n");
}









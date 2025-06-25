#pragma once





/*指定变量或函数存放的区域,注意看分散加载文件*/
//attribute:Normal memory
#define IRAM1_ATTR     __attribute__((section("IRAM1")))
//attribute:Core coupled memory
#define IRAM2_ATTR     __attribute__((section("IRAM2")))
//attribute:flash memory
#define IROM_ATTR      __attribute__((section("IROM")))
//attribute:手动设置对齐
#define ALIGN_ATTR(__BYTES) __attribute__((aligned(__BYTES)))
//中断服务函数标识
#define ISR_ATTR       __attribute__((used))

#define APPNAME                        "sQDFC"
#define HARDWARE_VERSION               "V2.0"
#define SOFTWARE_VERSION               "V1.0"

#define KEY_Pin                        GPIO_PIN_13
#define KEY_GPIO_Port                  GPIOC
#define ICM_INT_Pin                    GPIO_PIN_0
#define ICM_INT_GPIO_Port              GPIOC
#define FERAM_MISO_Pin                 GPIO_PIN_2
#define FERAM_MISO_GPIO_Port           GPIOC
#define FERAM_MOSI_Pin                 GPIO_PIN_3
#define FERAM_MOSI_GPIO_Port           GPIOC
#define DR_TX_Pin                      GPIO_PIN_0
#define DR_TX_GPIO_Port                GPIOA
#define DR_RX_Pin                      GPIO_PIN_1
#define DR_RX_GPIO_Port                GPIOA
#define OP_TX_Pin                      GPIO_PIN_2
#define OP_TX_GPIO_Port                GPIOA
#define OP_RX_Pin                      GPIO_PIN_3
#define OP_RX_GPIO_Port                GPIOA
#define ICM_CS_Pin                     GPIO_PIN_4
#define ICM_CS_GPIO_Port               GPIOA
#define IMU_SCK_Pin                    GPIO_PIN_5
#define IMU_SCK_GPIO_Port              GPIOA
#define IMU_MISO_Pin                   GPIO_PIN_6
#define IMU_MISO_GPIO_Port             GPIOA
#define IMU_MOSI_Pin                   GPIO_PIN_7
#define IMU_MOSI_GPIO_Port             GPIOA
#define LIS3_CS_Pin                    GPIO_PIN_4
#define LIS3_CS_GPIO_Port              GPIOC
#define WS2812_Pin                     GPIO_PIN_0
#define WS2812_GPIO_Port               GPIOB
#define RUN_LED_Pin                    GPIO_PIN_1
#define RUN_LED_GPIO_Port              GPIOB
#define ERR_LED_Pin                    GPIO_PIN_2
#define ERR_LED_GPIO_Port              GPIOB
#define EX_TX_Pin                      GPIO_PIN_10
#define EX_TX_GPIO_Port                GPIOB
#define EX_RX_Pin                      GPIO_PIN_11
#define EX_RX_GPIO_Port                GPIOB
#define BKIN_Pin                       GPIO_PIN_12
#define BKIN_GPIO_Port                 GPIOB
#define FERAM_SCK_Pin                  GPIO_PIN_13
#define FERAM_SCK_GPIO_Port            GPIOB
#define FRONT_LIGHT_Pin                GPIO_PIN_14
#define FRONT_LIGHT_GPIO_Port          GPIOB
#define LANDING_LIGHT_Pin              GPIO_PIN_15
#define LANDING_LIGHT_GPIO_Port        GPIOB
#define GPS_TX_Pin                     GPIO_PIN_6
#define GPS_TX_GPIO_Port               GPIOC
#define GPS_RX_Pin                     GPIO_PIN_7
#define GPS_RX_GPIO_Port               GPIOC
#define ESC_LU1_Pin                    GPIO_PIN_8
#define ESC_LU1_GPIO_Port              GPIOA
#define ESC_RU2_Pin                    GPIO_PIN_9
#define ESC_RU2_GPIO_Port              GPIOA
#define ESC_LD3_Pin                    GPIO_PIN_10
#define ESC_LD3_GPIO_Port              GPIOA
#define ESC_RD4_Pin                    GPIO_PIN_11
#define ESC_RD4_GPIO_Port              GPIOA
#define SD_DETECT_Pin                  GPIO_PIN_12
#define SD_DETECT_GPIO_Port            GPIOA
#define IMU_HEATER_Pin                 GPIO_PIN_15
#define IMU_HEATER_GPIO_Port           GPIOA
#define BUZZER_Pin                     GPIO_PIN_3
#define BUZZER_GPIO_Port               GPIOB
#define FERAM_CS_Pin                   GPIO_PIN_4
#define FERAM_CS_GPIO_Port             GPIOB
#define DEBUG_TX_Pin                   GPIO_PIN_6
#define DEBUG_TX_GPIO_Port             GPIOB
#define DEBUG_RX_Pin                   GPIO_PIN_7
#define DEBUG_RX_GPIO_Port             GPIOB

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   Header for MPU6050 I2C angle reader on PB8/PB9
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define MPU6050_I2C_ADDRESS      (0x68 << 1)

typedef struct {
    float roll;
    float pitch;
    float yaw;
} MPU6050_Angle_t;

bool MPU6050_Init(void);
bool MPU6050_ReadAngles(MPU6050_Angle_t *angle);

/* Debug / raw output (visible as live expressions) */
extern volatile int16_t mpu_raw_ax;
extern volatile int16_t mpu_raw_ay;
extern volatile int16_t mpu_raw_az;
extern volatile int16_t mpu_raw_gx;
extern volatile int16_t mpu_raw_gy;
extern volatile int16_t mpu_raw_gz;
extern volatile float mpu_acc_x;
extern volatile float mpu_acc_y;
extern volatile float mpu_acc_z;
extern volatile float mpu_gyro_x;
extern volatile float mpu_gyro_y;
extern volatile float mpu_gyro_z;

/* Optional: allow swapping/inverting axes if sensor orientation differs */
void MPU6050_SetAxisMapping(int swapXY, int invertX, int invertY, int invertZ);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */

#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    float accel_gx, accel_gy, accel_gz;

    int16_t gyro_x, gyro_y, gyro_z;
    float gyro_dps_x, gyro_dps_y, gyro_dps_z;

    int16_t temp_raw;
    float temperature;

    float roll, pitch;
} MPU6050_Data_t;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t int_polarity);
void MPU6050_Process(void);
void MPU6050_HandleInterrupt(void);
void MPU6050_UpdateOrientation(float dt);

MPU6050_Data_t MPU6050_GetData(void);

#endif

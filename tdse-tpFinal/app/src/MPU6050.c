#include "MPU6050.h"
#include <stdio.h>
#include <math.h>

#define MPU6050_ADDR         (0x68 << 1)
#define PWR_MGMT_1_REG       0x6B
#define DATA_START_REG       0x3B
#define ACCEL_CONFIG_REG     0x1C
#define GYRO_CONFIG_REG      0x1B
#define INT_PIN_CFG_REG      0x37
#define INT_ENABLE_REG       0x38

static I2C_HandleTypeDef *mpu_i2c = NULL;
static MPU6050_Data_t mpu_data;
static volatile uint8_t data_ready = 0;

static float alpha = 0.9f; // filtro complementario
static uint8_t inicializado = 0;


HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t int_polarity)
{
    mpu_i2c = hi2c;
    HAL_StatusTypeDef ret;
    uint8_t data;

    uint8_t who_am_i;
    ret = HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, 0x75, 1, &who_am_i, 1, 100);
    if(ret == HAL_OK) {
        printf("WHO_AM_I = 0x%02X\n", who_am_i);
    } else {
        printf("Error leyendo WHO_AM_I\n");
    }

    // 1) Setear reloj a PLL con gyro X
    data = 0x01;
    ret = HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("Error escribiendo PWR_MGMT_1\n");
        return ret;
    }

    HAL_Delay(100);

    // 2) Configurar acelerometro a ±2g
    data = 0x00;
    ret = HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("Error escribiendo ACCEL_CONFIG\n");
        return ret;
    }

    // 3) Configurar giroscopio a ±250 °/s
    data = 0x00;
    ret = HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("Error escribiendo GYRO_CONFIG\n");
        return ret;
    }

    // 4) Leer registro INT_PIN_CFG
    ret = HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("Error leyendo INT_PIN_CFG\n");
        return ret;
    }

    // 5) Modificar bit 7 según int_polarity
    if (int_polarity == 0)
        data &= ~(1 << 7); // activo alto
    else
        data |= (1 << 7);  // activo bajo

    // 6) Escribir nuevamente INT_PIN_CFG
    ret = HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("Error escribiendo INT_PIN_CFG\n");
        return ret;
    }

    // 7) Activar interrupción Data Ready
    data = 0x01;
    ret = HAL_I2C_Mem_Write(mpu_i2c, MPU6050_ADDR, INT_ENABLE_REG, 1, &data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        printf("Error escribiendo INT_ENABLE\n");
        return ret;
    }

    return HAL_OK;
}


void MPU6050_HandleInterrupt(void) {
    data_ready = 1;
}

#include <time.h>  // o usa un timer hardware para medir dt

static uint32_t last_tick = 0;

void MPU6050_Process(void) {
    if (!data_ready) return;
    data_ready = 0;

    uint8_t buf[14];
    if (HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR, DATA_START_REG, 1, buf, 14, HAL_MAX_DELAY) != HAL_OK) {
        printf("Error leyendo datos del MPU6050\n");
        return;
    }

    // Leer datos crudos
    mpu_data.accel_x = (buf[0] << 8) | buf[1];
    mpu_data.accel_y = (buf[2] << 8) | buf[3];
    mpu_data.accel_z = (buf[4] << 8) | buf[5];
    mpu_data.temp_raw = (buf[6] << 8) | buf[7];
    mpu_data.gyro_x  = (buf[8]  << 8) | buf[9];
    mpu_data.gyro_y  = (buf[10] << 8) | buf[11];
    mpu_data.gyro_z  = (buf[12] << 8) | buf[13];

    // Convertir unidades
    mpu_data.accel_gx = mpu_data.accel_x * 9.80665f / 16384.0f;
    mpu_data.accel_gy = mpu_data.accel_y * 9.80665f / 16384.0f;
    mpu_data.accel_gz = mpu_data.accel_z * 9.80665f / 16384.0f;

    mpu_data.gyro_dps_x = mpu_data.gyro_x / 131.0f;
    mpu_data.gyro_dps_y = mpu_data.gyro_y / 131.0f;
    mpu_data.gyro_dps_z = mpu_data.gyro_z / 131.0f;

    mpu_data.temperature = mpu_data.temp_raw / 340.0f + 36.53f;

    // Obtener tiempo actual (ejemplo con HAL_GetTick)
    uint32_t current_tick = HAL_GetTick(); // ms desde inicio
    float dt = (last_tick == 0) ? 0.01f : (current_tick - last_tick) / 1000.0f;
    last_tick = current_tick;

    // Actualizar ángulos con filtro complementario
    MPU6050_UpdateOrientation(dt);

}



void MPU6050_UpdateOrientation(float dt) {
    // Ángulos por acelerómetro (en grados)
    float roll_acc  = atan2f(mpu_data.accel_gy, mpu_data.accel_gz) * 180.0f / M_PI;
    float pitch_acc = atan2f(-mpu_data.accel_gx, sqrtf(mpu_data.accel_gy * mpu_data.accel_gy + mpu_data.accel_gz * mpu_data.accel_gz)) * 180.0f / M_PI;

    // Primera lectura: inicializar ángulos con acelerómetro para evitar "saltos"
    if (!inicializado) {
    	mpu_data.roll = roll_acc;
    	mpu_data.pitch = pitch_acc;
        inicializado = 1;
    }

    // Integrar giroscopio (velocidad angular en grados/seg)
    mpu_data.roll  += mpu_data.gyro_dps_x * dt;
    mpu_data.pitch += mpu_data.gyro_dps_y * dt;

    // Aplicar filtro complementario
    mpu_data.roll  = alpha * mpu_data.roll  + (1.0f - alpha) * roll_acc;
    mpu_data.pitch = alpha * mpu_data.pitch + (1.0f - alpha) * pitch_acc;

    //printf("Roll: %.2f°, Pitch: %.2f°\n", mpu_data.roll, mpu_data.pitch);
}



MPU6050_Data_t MPU6050_GetData(void) {
	//printf("Roll: %.2f°, Pitch: %.2f°\n", mpu_data.roll, mpu_data.pitch);
    return mpu_data;
}

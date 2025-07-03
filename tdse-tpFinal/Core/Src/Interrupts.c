#include "Interrupts.h"
#include "MPU6050.h"
#include "main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT_MPU6050_Pin)
    {
        MPU6050_HandleInterrupt();
    }
}

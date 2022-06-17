/*
    stMPU6050.h
    A scuffed header file library to use MPU6050 with STM32
*/
#ifndef __STMPU6050_H
#define __STMPU6050_H

#include "i2c.h"

// Include this line based on the STM32 Microcontroller you are using
#include "stm32f4xx.h"  // The Development Board being used is STM32F407-DISC


/* Register Addresses */

// I2C Bus Address for MPU
#ifdef ENABLE_ADO // Put #define ENABLE_AD0 if you have set AD0 pin to high
    #define MPU_I2C_ADDR    0x69
#else
    #define MPU_I2C_ADDR    0x68
#endif

// MPU Config
#define GYRO_CONFIG_ADDR    0x1B
#define ACCEL_CONFIG_ADDR   0x1C
#define POWER_MGMT_ADDR     0x6B

// Accelerometer
#define ACCEL_XOUT_H_ADDR   0x3B
#define ACCEL_XOUT_L_ADDR   0x3C
#define ACCEL_YOUT_H_ADDR   0x3D
#define ACCEL_YOUT_L_ADDR   0x3E
#define ACCEL_ZOUT_H_ADDR   0x3F
#define ACCEL_ZOUT_L_ADDR   0x40

// Gyrometer
#define GYRO_XOUT_H_ADDR    0x43
#define GYRO_XOUT_L_ADDR    0x44
#define GYRO_YOUT_H_ADDR    0x45
#define GYRO_YOUT_L_ADDR    0x46
#define GYRO_ZOUT_H_ADDR    0x47
#define GYRO_ZOUT_L_ADDR    0x48

// Temperature Sensor
#define TEMP_OUT_H_ADDR     0x65
#define TEMP_OUT_L_ADDR     0x66 


/* Function Prototypes */

void initMPU(I2C_HandleTypeDef *hi2c); // Initialize MPU for use
void readRawAccelVal(I2C_HandleTypeDef *hi2c, uint16_t *accel_val);
void readEawGyroVal(I2C_HandleTypeDef *hi2c, uint16_t *gyro_val);

/* Function Definitiona */

void initMPU(I2C_HandleTypeDef *hi2c) {
    uint8_t data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU_I2C_ADDR << 1, POWER_MGMT_ADDR, 1, &data, 1, 50);     // Wakeup MPU6050
}

void readRawAccelVal(I2C_HandleTypeDef *hi2c, uint16_t *accel_val) {
    uint8_t temp8bit;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, ACCEL_XOUT_H_ADDR, 1, &temp8bit, 1, 50);
    accel_val[0] = temp8bit << 8;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, ACCEL_XOUT_L_ADDR, 1, &temp8bit, 1, 50);
    accel_val[0] = accel_val[0] | temp8bit;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, ACCEL_YOUT_H_ADDR, 1, &temp8bit, 1, 50);
    accel_val[1] = temp8bit << 8;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, ACCEL_YOUT_L_ADDR, 1, &temp8bit, 1, 50);
    accel_val[1] = accel_val[1] | temp8bit;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, ACCEL_ZOUT_H_ADDR, 1, &temp8bit, 1, 50);
    accel_val[2] = temp8bit << 8;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, ACCEL_ZOUT_L_ADDR, 1, &temp8bit, 1, 50);
    accel_val[2] = accel_val[2] | temp8bit;
}

void readRawGyroVal(I2C_HandleTypeDef *hi2c, uint16_t *gyro_val) {
    uint8_t temp8bit;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, GYRO_XOUT_H_ADDR, 1, &temp8bit, 1, 50);
    gyro_val[0] = temp8bit << 8;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, GYRO_XOUT_L_ADDR, 1, &temp8bit, 1, 50);
    gyro_val[0] = gyro_val[0] | temp8bit;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, GYRO_YOUT_H_ADDR, 1, &temp8bit, 1, 50);
    gyro_val[1] = temp8bit << 8;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, GYRO_YOUT_L_ADDR, 1, &temp8bit, 1, 50);
    gyro_val[1] = gyro_val[1] | temp8bit;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, GYRO_ZOUT_H_ADDR, 1, &temp8bit, 1, 50);
    gyro_val[2] = temp8bit << 8;
    HAL_I2C_Mem_Read(hi2c, MPU_I2C_ADDR << 1 | 0x01, GYRO_ZOUT_L_ADDR, 1, &temp8bit, 1, 50);
    gyro_val[2] = gyro_val[2] | temp8bit;
}


#endif

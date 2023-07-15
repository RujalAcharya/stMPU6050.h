/*
    stMPU6050.h
    A scuffed header file library to use MPU6050 with STM32
*/
#ifndef __STMPU6050_H
#define __STMPU6050_H

#include "i2c.h"
#include <math.h>
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

/* Configuration of MPU6050 */

#define PI 3.1415

// Set the gyro full scale to 250, 500, 1000 or 2000 deg/s
typedef enum {
    FS_250_DEG_S    =   0x00,
    FS_500_DEG_S    =   0x08,
    FS_1000_DEG_S   =   0x10,
    FS_2000_DEG_S   =   0x18
} gyroFullScaleRange;

// Set the accelerometer full scale to 2, 4, 8 or 16 g
typedef enum {
    FS_2_G          =   0x00,
    FS_4_G          =   0x08,
    FS_8_G          =   0x10,
    FS_16_G         =   0x18
} accelFullScaleRange;

// NOTE: The gyro sensitivities are 131, 65.5, 32.8 and 16.4. Since enum doesn't accept float, they are 
// simply multiplied by 10 and then later divided by 10.0 (as a silly hack).. There are better ways to 
// implement this
typedef enum {
    SEN_250_DEG_S   =   1310,
    SEN_500_DEG_S   =   655,
    SEN_1000_DEG_S  =   328,
    SEN_2000_DEG_S  =   164
} gyroLSBSensitivity;

typedef enum {
    SEN_2_G         =   16384,
    SEN_4_G         =   8192,
    SEN_8_G         =   4096,
    SEN_16_G        =   2048
} accelLSBSensitivity;

typedef struct{
    float elapsedTime;
    float currentTime;
    float previousTime;
}time_val;

typedef struct {
    accelFullScaleRange accelFullScale;
    gyroFullScaleRange gyroFullScale;
    accelLSBSensitivity accelSensitivity;
    gyroLSBSensitivity gyroSensitivity;
    float AccErrorX;
    float AccErrorY;
    float AccErrorZ;
    float GyroErrorX;
    float GyroErrorY;
    float GyroErrorYZ;
    time_val time;
} MPUConfigHandle;


/* Function Prototypes */

void initMPU(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu); // Initialize MPU for use
void readRawAccelVal(I2C_HandleTypeDef *hi2c, uint16_t *accel_val);
void readRawGyroVal(I2C_HandleTypeDef *hi2c, uint16_t *gyro_val);
void readScaledAcclerVal(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu, float *accler_val);
void readScaledGyroVal(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu, float *gyro_val);
void getAngleByAccler(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu, double *roll, double *pitch);
void getAngleByGyro(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu, double *roll,double *pitch, double *yaw);
void calculateImuError(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu);

/* Function Definitions */

void initMPU(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu) {
    HAL_I2C_Mem_Write(hi2c, MPU_I2C_ADDR << 1, POWER_MGMT_ADDR, 1, 0x00, 1, 50);     // Wakeup MPU6050 from sleep mode

    HAL_I2C_Mem_Write(hi2c, MPU_I2C_ADDR << 1, ACCEL_CONFIG_ADDR, 1, (uint8_t) hmpu->accelFullScale, 1, 50); // Setup accelerometer full scale

    // Setuo accelerometer sensitivity according to its full scale
    switch (hmpu->accelFullScale) {
    case FS_2_G:
        hmpu->accelSensitivity = SEN_2_G;    
        break;
    case FS_4_G:
        hmpu->accelSensitivity = SEN_4_G;    
        break;
    case FS_8_G:
        hmpu->accelSensitivity = SEN_8_G;    
        break;
    case FS_16_G:
        hmpu->accelSensitivity = SEN_16_G;    
        break;
    default:
        break;
    }

    HAL_I2C_Mem_Write(hi2c, MPU_I2C_ADDR << 1, GYRO_CONFIG_ADDR, 1, (uint8_t) hmpu->gyroFullScale, 1, 50); // Setup gyro full scale

    // Setup gyro sensitivity according to its full scale
    switch (hmpu->gyroFullScale) {
    case FS_250_DEG_S:
        hmpu->gyroSensitivity = SEN_250_DEG_S;
        break;
    case FS_500_DEG_S:
        hmpu->gyroSensitivity = SEN_500_DEG_S;
        break;
    case FS_1000_DEG_S:
        hmpu->gyroSensitivity = SEN_1000_DEG_S;
        break;
    case FS_2000_DEG_S:
        hmpu->gyroSensitivity = SEN_2000_DEG_S;
        break;
    default:
        break;
    }

    hmpu->time.currentTime = HAL_GetTick();
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

void readScaledAcclerVal(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu, float *accler_val){
    uint16_t rawAccVal[3];
    readRawAccelVal(hi2c,rawAccVal);
    for(int i = 0; i<3; i++){
        accler_val[i] = ((int16_t)rawAccVal[i])/(hmpu->accelSensitivity * 1.0);
    }
}

void readScaledGyroVal(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu, float *gyro_val){
    uint16_t rawGyroVal[3];
    readRawGyroVal(hi2c,rawGyroVal);
    for(int i = 0; i<3; i++){
        gyro_val[i] = ((int16_t)rawGyroVal[i])/(hmpu->gyroSensitivity / 10.0);
    }
}


void getAngleByAccler(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu, double *roll, double *pitch){
    float accler_val[3];
    readScaledAcclerVal(hi2c,hmpu,accler_val);
    *roll = asin(accler_val[0]/sqrt(accler_val[0]*accler_val[0]+accler_val[1]*accler_val[1]+accler_val[2]*accler_val[2]))*180/PI;
    *pitch = atan2(accler_val[1],accler_val[2])*180/PI;
}

void getAngleByGyro(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu, double *roll,double *pitch, double *yaw){
    float gyro_val[3];
    readScaledGyroVal(hi2c,hmpu,gyro_val);
    hmpu->time.previousTime = hmpu->time.currentTime;
    hmpu->time.currentTime = HAL_GetTick();
    hmpu->time.elapsedTime = (hmpu->time.currentTime-hmpu->time.previousTime)/1000;

    *roll = *roll+gyro_val[0]*hmpu->time.elapsedTime;
    *pitch = *pitch +gyro_val[1]*hmpu->time.elapsedTime;
    *yaw = *yaw+gyro_val[2]*hmpu->time.elapsedTime;
}
void getAngleByFilter(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu,double *roll , double *pitch, double *yaw){
    double accler_roll,accler_pitch,gyro_roll,gyro_pitch,gyro_yaw;
    getAngleByAccler(hi2c,hmpu,&accler_roll, &accler_pitch);
    getAngleByGyro(hi2c,hmpu,&gyro_roll, &gyro_pitch, &gyro_pitch);
    *roll = 0.5*gyro_roll+0.5*accler_roll;
    *pitch = 0.5*gyro_pitch+0.5*accler_pitch;
    *yaw = gyro_yaw; 
}
void calculateImuError(I2C_HandleTypeDef *hi2c, MPUConfigHandle *hmpu){
    hmpu->AccErrorX = 0;
	hmpu->AccErrorY = 0;
	hmpu->GyroErrorX = 0;
	hmpu->GyroErrorY = 0;
	hmpu->GyroErrorZ = 0;
    int c =0;
    while (c<200)
    {
        uint16_t accler_val[3];
        int16_t accx,accy,accz;
        readRawAccelVal(hi2c,accler_val);
        accx = (int16_t) accler_val[0];
        accy = (int16_t) accler_val[1];
        accz = (int16_t) accler_val[2];
        hmpu->AccErrorX = AccErrorX + ((atan((accy) / sqrt(pow((accx), 2) + pow((accz), 2))) * 180 / PI));
		hmpu->AccErrorY = AccErrorY + ((atan(-1 * (accx) / sqrt(pow((accy), 2) + pow((accz), 2))) * 180 / PI));
		c++;
    }
    c = 0;
    hmpu->AccErrorX =hmpu->AccErrorX /200;
    hmpu->AccErrorY =hmpu->AccErrorY /200;

    while (c<200)
    {
        float gyro_val[3];
        readScaledGyroVal(hi2c,hmpu,gyro_val);
        hmpu->GyroErrorX = hmpu->GyroErrorX + (gyro_val[0] / 131.0);
		hmpu->GyroErrorY = hmpu->GyroErrorY + (gyro_val[1] / 131.0);
		hmpu->GyroErrorZ = hmpu->GyroErrorZ + (gyro_val[2] / 131.0);
    }
    hmpu->GyroErrorX = hmpu->GyroErrorX/200;
	hmpu->GyroErrorY = hmpu->GyroErrorY/200;
	hmpu->GyroErrorZ = hmpu->GyroErrorZ/200;
}

#endif
 

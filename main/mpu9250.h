#ifndef MPU9250_H
#define MPU9250_H

#include "driver/i2c.h"
#include "esp_err.h"

#define MPU9250_ADDRESS      0x68
#define MPU9250_WHO_AM_I     0x75
#define MPU9250_PWR_MGMT_1   0x6B
#define MPU9250_CONFIG       0x1A
#define MPU9250_GYRO_CONFIG  0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_XOUT_H 0x3B

// 添加磁力计寄存器定义
#define AK8963_ADDRESS      0x0C
#define AK8963_WHO_AM_I    0x00
#define AK8963_CNTL1       0x0A
#define AK8963_ASAX        0x10
#define AK8963_ST1         0x02
#define AK8963_HXL         0x03

esp_err_t mpu9250_init(void);
esp_err_t mpu9250_read_raw_data(int16_t* ax, int16_t* ay, int16_t* az, 
    int16_t* gx, int16_t* gy, int16_t* gz);
void mpu9250_get_angles(float *roll, float *pitch, float *yaw);

void mpu9250_read_mag(int16_t* mx, int16_t* my, int16_t* mz);
esp_err_t read_bytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
void mpu9250_calibrate(void);
esp_err_t mpu9250_read_id(uint8_t *id);
esp_err_t mpu9250_read_accel(float *ax, float *ay, float *az);
esp_err_t mpu9250_get_accel(float *ax, float *ay, float *az);
esp_err_t mpu9250_get_gyro(float *gx, float *gy, float *gz);


#endif
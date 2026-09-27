#ifndef MPU6050_ADAPTER_H
#define MPU6050_ADAPTER_H
#include <stdint.h>
#include "sensor_types.h"
typedef struct { int16_t acc[3], temp, gyro[3]; } mpu6050_raw_t;
typedef struct {
    float acc_lsb_per_g, gyro_lsb_per_dps;
    float acc_bias[3], gyro_bias[3];
    float acc_M[9], gyro_M[9];
} mpu6050_calibration_t;
mpu6050_calibration_t mpu6050_default_calibration(void);
void mpu6050_convert(const mpu6050_raw_t *raw, uint64_t timestamp_us,
                     const mpu6050_calibration_t *cal, imu_sample_t *out,
                     float *temperature_c);
#endif


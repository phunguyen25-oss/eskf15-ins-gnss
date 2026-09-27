#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H
#include <stdint.h>
typedef struct { uint64_t t_measure_us; float acc[3], gyro[3]; } imu_sample_t;
typedef struct { uint64_t t_measure_us; float pos[3], vel[3], pos_var[3], vel_var[3]; } gps_sample_t;
typedef struct { uint64_t t_measure_us; float altitude, variance; } baro_sample_t;
typedef struct { uint64_t t_measure_us; float field[3], variance[3]; } mag_sample_t;
#endif


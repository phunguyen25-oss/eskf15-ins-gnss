#include "mpu6050_adapter.h"
#include <string.h>

mpu6050_calibration_t mpu6050_default_calibration(void){
    mpu6050_calibration_t c; memset(&c,0,sizeof(c));
    c.acc_lsb_per_g=16384.0f; c.gyro_lsb_per_dps=131.0f;
    c.acc_M[0]=c.acc_M[4]=c.acc_M[8]=1.0f;
    c.gyro_M[0]=c.gyro_M[4]=c.gyro_M[8]=1.0f; return c;
}
void mpu6050_convert(const mpu6050_raw_t *r,uint64_t ts,const mpu6050_calibration_t *c,
                     imu_sample_t *o,float *temp){
    const float g=9.80665f, d2r=0.0174532925199433f; float a[3],w[3];
    for(int i=0;i<3;i++){ a[i]=((float)r->acc[i]/c->acc_lsb_per_g)*g-c->acc_bias[i];
                          w[i]=((float)r->gyro[i]/c->gyro_lsb_per_dps)*d2r-c->gyro_bias[i]; }
    for(int i=0;i<3;i++){
        o->acc[i]=c->acc_M[3*i]*a[0]+c->acc_M[3*i+1]*a[1]+c->acc_M[3*i+2]*a[2];
        o->gyro[i]=c->gyro_M[3*i]*w[0]+c->gyro_M[3*i+1]*w[1]+c->gyro_M[3*i+2]*w[2];
    }
    o->t_measure_us=ts; if(temp) *temp=(float)r->temp/340.0f+36.53f;
}


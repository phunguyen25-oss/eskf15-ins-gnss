#include "eskf15.h"
#include "app_eskf.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static eskf15_t make_filter(void){
    eskf15_t s; float z[3]={0},q[4]={1,0,0,0},pd[ESKF_N];
    for(int i=0;i<ESKF_N;i++) pd[i]=1.0f;
    eskf_config_t c=eskf15_default_config(); c.sigma_acc=c.sigma_gyro=c.sigma_ba=c.sigma_bg=0;
    eskf15_init(&s,z,z,q,z,z,pd,&c); return s;
}
static app_eskf_t app;
int main(void){
    eskf15_t s=make_filter(); float a[3]={1,0,-9.80665f},w[3]={0};
    assert(eskf15_predict(&s,a,w,0.01f)==ESKF_OK);
    assert(fabsf(s.p[0]-0.5f*0.01f*0.01f)<1e-7f);
    assert(fabsf(s.v[0]-0.01f)<1e-6f);
    for(int k=0;k<1000;k++){
        float wr[3]={0.4f,-0.2f,0.1f}; assert(eskf15_predict(&s,a,wr,0.001f)==ESKF_OK);
    }
    float qn=sqrtf(s.q[0]*s.q[0]+s.q[1]*s.q[1]+s.q[2]*s.q[2]+s.q[3]*s.q[3]);
    assert(fabsf(qn-1.0f)<1e-5f); assert(eskf15_covariance_cholesky_ok(&s));
    float oldp=s.p[0],zp[3]={1000,1000,1000},zv[3]={0},pv[3]={0.01f,0.01f,0.01f},vv[3]={1,1,1},l[3]={0};
    assert(eskf15_update_gps(&s,zp,zv,pv,vv,l)!=ESKF_NUMERIC_FAULT);
    assert(fabsf(s.p[0]-oldp)<1.0f); /* gated outlier must not teleport state */
    app_eskf_init(&app);
    for(uint64_t t=1000;t<=20000;t+=1000){
        imu_sample_t m={0}; m.t_measure_us=t; m.acc[2]=-9.80665f;
        assert(app_eskf_push_imu(&app,&m)==ESKF_OK);
    }
    baro_sample_t b={0}; b.t_measure_us=5000; b.altitude=0; b.variance=0.2f;
    assert(app_eskf_push_baro(&app,&b)==ESKF_OK);
    assert(app.late_measurements==0);
    puts("All native ESKF tests passed."); return 0;
}

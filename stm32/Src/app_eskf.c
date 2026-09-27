#include "app_eskf.h"
#include <string.h>

typedef enum { MEAS_GPS,MEAS_BARO,MEAS_MAG } meas_kind_t;
static void history_pack(eskf_history_entry_t *e,const eskf15_t *s,const imu_sample_t *imu){
    e->t_us=imu->t_measure_us; e->imu=*imu; e->valid=true;
    memcpy(e->p,s->p,sizeof(e->p)); memcpy(e->v,s->v,sizeof(e->v)); memcpy(e->q,s->q,sizeof(e->q));
    memcpy(e->ba,s->ba,sizeof(e->ba)); memcpy(e->bg,s->bg,sizeof(e->bg)); memcpy(e->omega,s->omega,sizeof(e->omega));
    memcpy(e->P,s->P,sizeof(e->P));
}
static void history_restore(eskf15_t *s,const eskf_history_entry_t *e){
    memcpy(s->p,e->p,sizeof(e->p)); memcpy(s->v,e->v,sizeof(e->v)); memcpy(s->q,e->q,sizeof(e->q));
    memcpy(s->ba,e->ba,sizeof(e->ba)); memcpy(s->bg,e->bg,sizeof(e->bg)); memcpy(s->omega,e->omega,sizeof(e->omega));
    memcpy(s->P,e->P,sizeof(e->P));
}
static void history_push(app_eskf_t *a,const imu_sample_t *imu){
    history_pack(&a->history[a->history_head],&a->filter,imu);
    a->history_head=(uint16_t)((a->history_head+1u)%ESKF_HISTORY_LEN);
    if(a->history_count<ESKF_HISTORY_LEN) a->history_count++;
}
static void refresh_newest(app_eskf_t *a){
    if(a->history_count){
        uint16_t i=(uint16_t)((a->history_head+ESKF_HISTORY_LEN-1u)%ESKF_HISTORY_LEN);
        history_pack(&a->history[i],&a->filter,&a->history[i].imu);
    }
}
static eskf_status_t apply_measurement(app_eskf_t *a,meas_kind_t kind,const void *vp){
    if(kind==MEAS_GPS){ const gps_sample_t *m=(const gps_sample_t *)vp;
        return eskf15_update_gps(&a->filter,m->pos,m->vel,m->pos_var,m->vel_var,a->gps_lever_b); }
    if(kind==MEAS_BARO){ const baro_sample_t *m=(const baro_sample_t *)vp;
        return eskf15_update_baro(&a->filter,m->altitude,m->variance); }
    const mag_sample_t *m=(const mag_sample_t *)vp;
    return eskf15_update_mag(&a->filter,m->field,a->mag_n,m->variance);
}
static eskf_status_t delayed_update(app_eskf_t *a,uint64_t t,meas_kind_t kind,const void *m){
    if(!a->history_count) return ESKF_LATE_MEASUREMENT;
    uint16_t newest=(uint16_t)((a->history_head+ESKF_HISTORY_LEN-1u)%ESKF_HISTORY_LEN), idx=newest;
    bool found=false;
    for(uint16_t n=0;n<a->history_count;n++){
        if(a->history[idx].valid&&a->history[idx].t_us<=t){ found=true; break; }
        idx=(uint16_t)((idx+ESKF_HISTORY_LEN-1u)%ESKF_HISTORY_LEN);
    }
    if(!found){ a->late_measurements++; return ESKF_LATE_MEASUREMENT; }
    uint32_t accepted=a->filter.accepted,rejected=a->filter.rejected;
    history_restore(&a->filter,&a->history[idx]);
    a->filter.accepted=accepted; a->filter.rejected=rejected;
    eskf_status_t update_status=apply_measurement(a,kind,m);
    history_pack(&a->history[idx],&a->filter,&a->history[idx].imu);
    uint64_t previous=a->history[idx].t_us;
    while(idx!=newest){
        idx=(uint16_t)((idx+1u)%ESKF_HISTORY_LEN);
        float dt=(float)(a->history[idx].t_us-previous)*1e-6f;
        eskf_status_t st=eskf15_predict(&a->filter,a->history[idx].imu.acc,a->history[idx].imu.gyro,dt);
        if(st!=ESKF_OK) return st;
        previous=a->history[idx].t_us;
        history_pack(&a->history[idx],&a->filter,&a->history[idx].imu);
    }
    return update_status;
}

void app_eskf_init(app_eskf_t *a){
    memset(a,0,sizeof(*a)); float z[3]={0},q[4]={1,0,0,0};
    float pd[ESKF_N]={1,1,1,1,1,1,0.03f,0.03f,0.03f,0.01f,0.01f,0.01f,0.001f,0.001f,0.001f};
    eskf_config_t c=eskf15_default_config(); eskf15_init(&a->filter,z,z,q,z,z,pd,&c);
    eskf_health_init(&a->health); a->mag_n[0]=0.42f; a->mag_n[1]=0.08f; a->mag_n[2]=0.90f;
}
eskf_status_t app_eskf_push_imu(app_eskf_t *a,const imu_sample_t *m){
    if(!a||!m) return ESKF_BAD_ARGUMENT;
    if(a->last_imu_us==0){ a->last_imu_us=m->t_measure_us; history_push(a,m); return ESKF_OK; }
    if(m->t_measure_us<=a->last_imu_us) return ESKF_BAD_DT;
    float dt=(float)(m->t_measure_us-a->last_imu_us)*1e-6f; a->last_imu_us=m->t_measure_us;
    eskf_status_t st=eskf15_predict(&a->filter,m->acc,m->gyro,dt);
    if(st==ESKF_OK) history_push(a,m);
    return st;
}
eskf_status_t app_eskf_push_gps(app_eskf_t *a,const gps_sample_t *m){
    if(!a||!m) return ESKF_BAD_ARGUMENT;
    if(a->last_imu_us&&m->t_measure_us+2000u<a->last_imu_us) return delayed_update(a,m->t_measure_us,MEAS_GPS,m);
    eskf_status_t st=apply_measurement(a,MEAS_GPS,m); refresh_newest(a); return st;
}
eskf_status_t app_eskf_push_baro(app_eskf_t *a,const baro_sample_t *m){
    if(!a||!m) return ESKF_BAD_ARGUMENT;
    if(a->last_imu_us&&m->t_measure_us+2000u<a->last_imu_us) return delayed_update(a,m->t_measure_us,MEAS_BARO,m);
    eskf_status_t st=apply_measurement(a,MEAS_BARO,m); refresh_newest(a); return st;
}
eskf_status_t app_eskf_push_mag(app_eskf_t *a,const mag_sample_t *m){
    if(!a||!m) return ESKF_BAD_ARGUMENT;
    if(a->last_imu_us&&m->t_measure_us+2000u<a->last_imu_us) return delayed_update(a,m->t_measure_us,MEAS_MAG,m);
    eskf_status_t st=apply_measurement(a,MEAS_MAG,m); refresh_newest(a); return st;
}

#include "eskf_health.h"

void eskf_health_init(eskf_health_monitor_t *h){
    h->consecutive_rejects=0; h->reject_limit=30;
    h->max_pos_variance=100.0f; h->max_att_variance=0.5f;
}
eskf_health_level_t eskf_health_check(eskf_health_monitor_t *h,const eskf15_t *s,
                                      eskf_status_t st,bool imu_timeout){
    if(imu_timeout||st==ESKF_NUMERIC_FAULT||st==ESKF_BAD_DT||!eskf15_is_finite(s)) return ESKF_HEALTH_FAULT;
    if(st==ESKF_REJECTED) h->consecutive_rejects++; else if(st==ESKF_OK) h->consecutive_rejects=0;
    if(h->consecutive_rejects>h->reject_limit) return ESKF_HEALTH_DEGRADED;
    for(int i=0;i<3;i++) if(s->P[i*ESKF_N+i]>h->max_pos_variance) return ESKF_HEALTH_DEGRADED;
    for(int i=6;i<9;i++) if(s->P[i*ESKF_N+i]>h->max_att_variance) return ESKF_HEALTH_DEGRADED;
    return ESKF_HEALTH_OK;
}


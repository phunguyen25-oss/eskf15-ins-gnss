#ifndef ESKF_HEALTH_H
#define ESKF_HEALTH_H
#include "eskf15.h"
typedef enum { ESKF_HEALTH_OK=0, ESKF_HEALTH_DEGRADED=1, ESKF_HEALTH_FAULT=2 } eskf_health_level_t;
typedef struct {
    uint32_t consecutive_rejects;
    uint32_t reject_limit;
    float max_pos_variance, max_att_variance;
} eskf_health_monitor_t;
void eskf_health_init(eskf_health_monitor_t *h);
eskf_health_level_t eskf_health_check(eskf_health_monitor_t *h,
                                      const eskf15_t *s, eskf_status_t last_status,
                                      bool imu_timed_out);
#endif


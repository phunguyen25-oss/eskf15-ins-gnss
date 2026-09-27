#ifndef APP_ESKF_H
#define APP_ESKF_H
#include "eskf15.h"
#include "eskf_health.h"
#include "sensor_types.h"
#ifndef ESKF_HISTORY_LEN
#define ESKF_HISTORY_LEN 64
#endif
typedef struct {
    uint64_t t_us;
    float p[3],v[3],q[4],ba[3],bg[3],omega[3];
    float P[ESKF_N*ESKF_N];
    imu_sample_t imu;
    bool valid;
} eskf_history_entry_t;
typedef struct {
    eskf15_t filter;
    eskf_health_monitor_t health;
    uint64_t last_imu_us;
    uint32_t late_measurements;
    float mag_n[3];
    float gps_lever_b[3];
    eskf_history_entry_t history[ESKF_HISTORY_LEN];
    uint16_t history_head, history_count;
} app_eskf_t;
void app_eskf_init(app_eskf_t *app);
eskf_status_t app_eskf_push_imu(app_eskf_t *app, const imu_sample_t *m);
eskf_status_t app_eskf_push_gps(app_eskf_t *app, const gps_sample_t *m);
eskf_status_t app_eskf_push_baro(app_eskf_t *app, const baro_sample_t *m);
eskf_status_t app_eskf_push_mag(app_eskf_t *app, const mag_sample_t *m);
#endif

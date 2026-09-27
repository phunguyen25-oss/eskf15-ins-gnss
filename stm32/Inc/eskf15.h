#ifndef ESKF15_H
#define ESKF15_H
#include <stdbool.h>
#include <stdint.h>

#define ESKF_N 15

typedef enum {
    ESKF_OK=0, ESKF_REJECTED=1, ESKF_BAD_ARGUMENT=-1,
    ESKF_BAD_DT=-2, ESKF_NUMERIC_FAULT=-3, ESKF_LATE_MEASUREMENT=-4
} eskf_status_t;

typedef struct {
    float gravity;
    float sigma_acc, sigma_gyro, sigma_ba, sigma_bg;
    float gate_1d, min_variance, max_dt;
} eskf_config_t;

typedef struct {
    float p[3], v[3], q[4], ba[3], bg[3];
    float omega[3];
    float P[ESKF_N*ESKF_N];
    /* Fixed scratch storage: no heap and bounded stack use. */
    float work_a[ESKF_N*ESKF_N];
    float work_b[ESKF_N*ESKF_N];
    uint32_t accepted, rejected;
    eskf_config_t cfg;
} eskf15_t;

eskf_config_t eskf15_default_config(void);
void eskf15_init(eskf15_t *s, const float p[3], const float v[3],
                 const float q[4], const float ba[3], const float bg[3],
                 const float Pdiag[ESKF_N], const eskf_config_t *cfg);
eskf_status_t eskf15_predict(eskf15_t *s, const float acc_m[3],
                             const float gyro_m[3], float dt);
eskf_status_t eskf15_update_gps(eskf15_t *s, const float pos[3],
                                const float vel[3], const float pos_var[3],
                                const float vel_var[3], const float lever_b[3]);
eskf_status_t eskf15_update_baro(eskf15_t *s, float altitude, float variance);
eskf_status_t eskf15_update_mag(eskf15_t *s, const float mag_b[3],
                                const float mag_n[3], const float variance[3]);
bool eskf15_covariance_cholesky_ok(const eskf15_t *s);
bool eskf15_is_finite(const eskf15_t *s);
#endif


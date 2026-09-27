#ifndef ESKF_MATH_H
#define ESKF_MATH_H
#include <stdbool.h>
void eskf_skew(const float v[3], float S[9]);
void eskf_quat_mul(const float a[4], const float b[4], float q[4]);
bool eskf_quat_normalize(float q[4]);
void eskf_quat_exp(const float phi[3], float q[4]);
void eskf_quat_to_rot(const float q[4], float R[9]);
bool eskf_all_finite(const float *x, int n);
#endif


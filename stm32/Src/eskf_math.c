#include "eskf_math.h"
#include <math.h>

void eskf_skew(const float v[3], float S[9]) {
    S[0]=0; S[1]=-v[2]; S[2]=v[1]; S[3]=v[2]; S[4]=0; S[5]=-v[0];
    S[6]=-v[1]; S[7]=v[0]; S[8]=0;
}
void eskf_quat_mul(const float a[4], const float b[4], float q[4]) {
    q[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    q[1]=a[0]*b[1]+b[0]*a[1]+a[2]*b[3]-a[3]*b[2];
    q[2]=a[0]*b[2]+b[0]*a[2]+a[3]*b[1]-a[1]*b[3];
    q[3]=a[0]*b[3]+b[0]*a[3]+a[1]*b[2]-a[2]*b[1];
}
bool eskf_quat_normalize(float q[4]) {
    float n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    if (!isfinite(n) || n<1e-8f) return false;
    for(int i=0;i<4;i++) q[i]/=n;
    return true;
}
void eskf_quat_exp(const float p[3], float q[4]) {
    float a=sqrtf(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
    if(a<1e-6f){ q[0]=1.0f-0.125f*a*a; q[1]=0.5f*p[0]; q[2]=0.5f*p[1]; q[3]=0.5f*p[2]; }
    else { float k=sinf(0.5f*a)/a; q[0]=cosf(0.5f*a); q[1]=k*p[0]; q[2]=k*p[1]; q[3]=k*p[2]; }
    (void)eskf_quat_normalize(q);
}
void eskf_quat_to_rot(const float q_[4], float R[9]) {
    float q[4]={q_[0],q_[1],q_[2],q_[3]}; (void)eskf_quat_normalize(q);
    float w=q[0],x=q[1],y=q[2],z=q[3];
    R[0]=1-2*(y*y+z*z); R[1]=2*(x*y-z*w); R[2]=2*(x*z+y*w);
    R[3]=2*(x*y+z*w); R[4]=1-2*(x*x+z*z); R[5]=2*(y*z-x*w);
    R[6]=2*(x*z-y*w); R[7]=2*(y*z+x*w); R[8]=1-2*(x*x+y*y);
}
bool eskf_all_finite(const float *x,int n){ for(int i=0;i<n;i++) if(!isfinite(x[i])) return false; return true; }

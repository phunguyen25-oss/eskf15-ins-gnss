#include "eskf15.h"
#include "eskf_math.h"
#include <math.h>
#include <string.h>

#define A(i,j) ((i)*ESKF_N+(j))

static void mat15_mul(float *C,const float *X,const float *Y){
    for(int i=0;i<ESKF_N;i++) for(int j=0;j<ESKF_N;j++){
        float v=0; for(int k=0;k<ESKF_N;k++) v+=X[A(i,k)]*Y[A(k,j)]; C[A(i,j)]=v;
    }
}
static void mat15_mul_bt(float *C,const float *X,const float *Y){
    for(int i=0;i<ESKF_N;i++) for(int j=0;j<ESKF_N;j++){
        float v=0; for(int k=0;k<ESKF_N;k++) v+=X[A(i,k)]*Y[A(j,k)]; C[A(i,j)]=v;
    }
}
static void symmetrize_floor(eskf15_t *s){
    for(int i=0;i<ESKF_N;i++){
        if(s->P[A(i,i)]<s->cfg.min_variance) s->P[A(i,i)]=s->cfg.min_variance;
        for(int j=i+1;j<ESKF_N;j++){
            float v=0.5f*(s->P[A(i,j)]+s->P[A(j,i)]); s->P[A(i,j)]=v; s->P[A(j,i)]=v;
        }
    }
}
static void mat3_vec(const float R[9],const float x[3],float y[3]){
    for(int i=0;i<3;i++) y[i]=R[3*i]*x[0]+R[3*i+1]*x[1]+R[3*i+2]*x[2];
}
static void cross3(const float a[3],const float b[3],float c[3]){
    c[0]=a[1]*b[2]-a[2]*b[1]; c[1]=a[2]*b[0]-a[0]*b[2]; c[2]=a[0]*b[1]-a[1]*b[0];
}
static void inject_reset(eskf15_t *s,const float dx[ESKF_N]){
    for(int i=0;i<3;i++){ s->p[i]+=dx[i]; s->v[i]+=dx[3+i]; s->ba[i]+=dx[9+i]; s->bg[i]+=dx[12+i]; }
    float dq[4],qn[4]; eskf_quat_exp(&dx[6],dq); eskf_quat_mul(s->q,dq,qn);
    memcpy(s->q,qn,sizeof(qn)); (void)eskf_quat_normalize(s->q);
    memset(s->work_a,0,sizeof(s->work_a));
    for(int i=0;i<ESKF_N;i++) s->work_a[A(i,i)]=1.0f;
    float S[9]; eskf_skew(&dx[6],S);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) s->work_a[A(6+i,6+j)]-=0.5f*S[3*i+j];
    mat15_mul(s->work_b,s->work_a,s->P);
    mat15_mul_bt(s->work_a,s->work_b,s->work_a);
    memcpy(s->P,s->work_a,sizeof(s->P)); symmetrize_floor(s);
}

eskf_config_t eskf15_default_config(void){
    eskf_config_t c={9.80665f,0.08f,0.00610865f,0.002f,0.000261799f,6.63f,1e-10f,0.05f}; return c;
}
void eskf15_init(eskf15_t *s,const float p[3],const float v[3],const float q[4],
                 const float ba[3],const float bg[3],const float pd[ESKF_N],const eskf_config_t *cfg){
    memset(s,0,sizeof(*s)); s->cfg=cfg?*cfg:eskf15_default_config();
    memcpy(s->p,p,12); memcpy(s->v,v,12); memcpy(s->q,q,16); memcpy(s->ba,ba,12); memcpy(s->bg,bg,12);
    (void)eskf_quat_normalize(s->q); for(int i=0;i<ESKF_N;i++) s->P[A(i,i)]=pd[i];
}

eskf_status_t eskf15_predict(eskf15_t *s,const float am[3],const float wm[3],float dt){
    if(!s||!am||!wm) return ESKF_BAD_ARGUMENT;
    if(!(dt>0.0f&&dt<=s->cfg.max_dt&&isfinite(dt))) return ESKF_BAD_DT;
    float a[3],w[3],half[3],full[3],dq[4],qmid[4],qnew[4],R[9],an[3];
    for(int i=0;i<3;i++){ a[i]=am[i]-s->ba[i]; w[i]=wm[i]-s->bg[i]; half[i]=0.5f*w[i]*dt; full[i]=w[i]*dt; }
    eskf_quat_exp(half,dq); eskf_quat_mul(s->q,dq,qmid); eskf_quat_to_rot(qmid,R);
    mat3_vec(R,a,an); an[2]+=s->cfg.gravity;
    for(int i=0;i<3;i++){ float vold=s->v[i]; s->p[i]+=vold*dt+0.5f*an[i]*dt*dt; s->v[i]=vold+an[i]*dt; s->omega[i]=w[i]; }
    eskf_quat_exp(full,dq); eskf_quat_mul(s->q,dq,qnew); memcpy(s->q,qnew,sizeof(qnew));
    if(!eskf_quat_normalize(s->q)) return ESKF_NUMERIC_FAULT;

    float *F=s->work_a,*F2=s->work_b; memset(F,0,sizeof(s->work_a));
    for(int i=0;i<3;i++) F[A(i,3+i)]=1.0f;
    float Sa[9],Sw[9],RSa[9]; eskf_skew(a,Sa); eskf_skew(w,Sw);
    for(int i=0;i<3;i++) for(int j=0;j<3;j++){
        RSa[3*i+j]=R[3*i]*Sa[j]+R[3*i+1]*Sa[3+j]+R[3*i+2]*Sa[6+j];
        F[A(3+i,6+j)]=-RSa[3*i+j]; F[A(3+i,9+j)]=-R[3*i+j];
        F[A(6+i,6+j)]=-Sw[3*i+j];
    }
    for(int i=0;i<3;i++) F[A(6+i,12+i)]=-1.0f;
    mat15_mul(F2,F,F);
    for(int i=0;i<ESKF_N;i++) for(int j=0;j<ESKF_N;j++)
        F[A(i,j)]=(i==j?1.0f:0.0f)+F[A(i,j)]*dt+0.5f*F2[A(i,j)]*dt*dt;
    mat15_mul(F2,F,s->P);
    mat15_mul_bt(s->P,F2,F);
    float qdiag[ESKF_N]={0};
    for(int i=3;i<6;i++) qdiag[i]=s->cfg.sigma_acc*s->cfg.sigma_acc*dt;
    for(int i=6;i<9;i++) qdiag[i]=s->cfg.sigma_gyro*s->cfg.sigma_gyro*dt;
    for(int i=9;i<12;i++) qdiag[i]=s->cfg.sigma_ba*s->cfg.sigma_ba*dt;
    for(int i=12;i<15;i++) qdiag[i]=s->cfg.sigma_bg*s->cfg.sigma_bg*dt;
    for(int i=0;i<ESKF_N;i++) s->P[A(i,i)]+=qdiag[i];
    symmetrize_floor(s);
    return eskf15_is_finite(s)?ESKF_OK:ESKF_NUMERIC_FAULT;
}

static eskf_status_t update_seq(eskf15_t *s,const float *res,const float *H,const float *var,int m){
    float dx[ESKF_N]={0},pht[ESKF_N],K[ESKF_N]; bool any=false;
    for(int z=0;z<m;z++){
        const float *h=&H[z*ESKF_N]; float innov=res[z];
        for(int i=0;i<ESKF_N;i++) innov-=h[i]*dx[i];
        for(int i=0;i<ESKF_N;i++){ pht[i]=0; for(int j=0;j<ESKF_N;j++) pht[i]+=s->P[A(i,j)]*h[j]; }
        float S=var[z]; for(int i=0;i<ESKF_N;i++) S+=h[i]*pht[i];
        float nis=innov*innov/S;
        if(!(isfinite(S)&&S>0&&isfinite(nis)&&nis<=s->cfg.gate_1d)){ s->rejected++; continue; }
        for(int i=0;i<ESKF_N;i++) K[i]=pht[i]/S;
        for(int i=0;i<ESKF_N;i++) for(int j=0;j<ESKF_N;j++)
            s->work_a[A(i,j)]=s->P[A(i,j)]-K[i]*pht[j]-pht[i]*K[j]+K[i]*S*K[j];
        memcpy(s->P,s->work_a,sizeof(s->P));
        for(int i=0;i<ESKF_N;i++) dx[i]+=K[i]*innov;
        s->accepted++; any=true;
    }
    if(any) inject_reset(s,dx); else symmetrize_floor(s);
    if(!eskf15_is_finite(s)) return ESKF_NUMERIC_FAULT;
    return any?ESKF_OK:ESKF_REJECTED;
}

eskf_status_t eskf15_update_gps(eskf15_t *s,const float zp[3],const float zv[3],
                                const float pv[3],const float vv[3],const float l[3]){
    if(!s||!zp||!zv||!pv||!vv||!l) return ESKF_BAD_ARGUMENT;
    float H[6*ESKF_N]={0},res[6],var[6],R[9],Rl[3],rot[3],Rrot[3],Sl[9],Srot[9];
    eskf_quat_to_rot(s->q,R); mat3_vec(R,l,Rl); cross3(s->omega,l,rot); mat3_vec(R,rot,Rrot);
    eskf_skew(l,Sl); eskf_skew(rot,Srot);
    for(int i=0;i<3;i++){
        res[i]=zp[i]-(s->p[i]+Rl[i]); res[3+i]=zv[i]-(s->v[i]+Rrot[i]); var[i]=pv[i]; var[3+i]=vv[i];
        H[i*ESKF_N+i]=1; H[(3+i)*ESKF_N+3+i]=1;
        for(int j=0;j<3;j++){
            float rs=0,rr=0; for(int k=0;k<3;k++){ rs+=R[3*i+k]*Sl[3*k+j]; rr+=R[3*i+k]*Srot[3*k+j]; }
            H[i*ESKF_N+6+j]=-rs; H[(3+i)*ESKF_N+6+j]=-rr; H[(3+i)*ESKF_N+12+j]=rs;
        }
    }
    return update_seq(s,res,H,var,6);
}
eskf_status_t eskf15_update_baro(eskf15_t *s,float altitude,float variance){
    if(!s||!(variance>0)) return ESKF_BAD_ARGUMENT;
    float H[ESKF_N]={0},res=altitude+s->p[2]; H[2]=-1; return update_seq(s,&res,H,&variance,1);
}
eskf_status_t eskf15_update_mag(eskf15_t *s,const float mb[3],const float mn[3],const float var[3]){
    if(!s||!mb||!mn||!var) return ESKF_BAD_ARGUMENT;
    float R[9],pred[3],H[3*ESKF_N]={0},res[3],Sp[9]; eskf_quat_to_rot(s->q,R);
    for(int i=0;i<3;i++) pred[i]=R[i]*mn[0]+R[3+i]*mn[1]+R[6+i]*mn[2];
    eskf_skew(pred,Sp);
    for(int i=0;i<3;i++){ res[i]=mb[i]-pred[i]; for(int j=0;j<3;j++) H[i*ESKF_N+6+j]=Sp[3*i+j]; }
    return update_seq(s,res,H,var,3);
}
bool eskf15_is_finite(const eskf15_t *s){
    return s&&eskf_all_finite(s->p,3)&&eskf_all_finite(s->v,3)&&eskf_all_finite(s->q,4)&&
        eskf_all_finite(s->ba,3)&&eskf_all_finite(s->bg,3)&&eskf_all_finite(s->P,ESKF_N*ESKF_N);
}
bool eskf15_covariance_cholesky_ok(const eskf15_t *s){
    float L[ESKF_N*ESKF_N]={0};
    for(int i=0;i<ESKF_N;i++) for(int j=0;j<=i;j++){
        float sum=s->P[A(i,j)]; for(int k=0;k<j;k++) sum-=L[A(i,k)]*L[A(j,k)];
        if(i==j){ if(!(sum>0&&isfinite(sum))) return false; L[A(i,j)]=sqrtf(sum); }
        else L[A(i,j)]=sum/L[A(j,j)];
    }
    return true;
}

function s = eskf_predict(s,acc_m,gyro_m,dt)
if ~(isfinite(dt) && dt>0 && dt<=s.cfg.max_dt)
    error('ESKF:badDt','dt is outside (0,max_dt].');
end
a=acc_m(:)-s.ba; w=gyro_m(:)-s.bg;
q0=s.q; v0=s.v;
qmid=quat_mul(q0,quat_exp(0.5*w*dt));
Rmid=quat_to_rot(qmid);
an=Rmid*a+[0;0;s.cfg.g];
s.p=s.p+v0*dt+0.5*an*dt*dt;
s.v=v0+an*dt;
s.q=quat_mul(q0,quat_exp(w*dt)); s.q=s.q/norm(s.q);
s.omega=w;
[F,G,Qc]=eskf_FG(s,a,w,Rmid);
[Phi,Qd]=eskf_discretize(F,G,Qc,dt,s.cfg.use_van_loan);
s.P=Phi*s.P*Phi'+Qd;
s.P=0.5*(s.P+s.P');
d=diag(s.P); d=max(d,s.cfg.min_variance);
s.P(1:16:end)=d;
end


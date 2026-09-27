function d = simulate_sensors(tr,dt,seed)
if nargin<3, seed=7; end
rng(seed,'twister'); n=numel(tr.t);
c=eskf_default_config();
d.acc=tr.f+tr.ba+(c.sigma_acc/sqrt(dt))*randn(3,n);
d.gyro=tr.omega+tr.bg+(c.sigma_gyro/sqrt(dt))*randn(3,n);
d.gps_mask=false(1,n); d.gps_mask(1:round(0.2/dt):end)=true;
d.baro_mask=false(1,n); d.baro_mask(1:round(0.05/dt):end)=true;
d.mag_mask=false(1,n); d.mag_mask(1:round(0.05/dt):end)=true;
d.gps_p=tr.p+0.7*randn(3,n); d.gps_v=tr.v+0.15*randn(3,n);
d.baro=-tr.p(3,:)+0.4*randn(1,n);
d.mag_n=[0.42;0.08;0.90]; d.mag_n=d.mag_n/norm(d.mag_n);
d.mag=zeros(3,n);
for k=1:n
    d.mag(:,k)=quat_to_rot(tr.q(:,k))'*d.mag_n+0.015*randn(3,1);
end
% One deliberate GPS outlier to exercise innovation gating.
k=find(d.gps_mask,1,'last'); d.gps_p(:,k)=d.gps_p(:,k)+[35;-20;10];
d.Rp=(0.7^2)*eye(3); d.Rv=(0.15^2)*eye(3);
d.Rbaro=0.4^2; d.Rmag=(0.015^2)*eye(3);
end


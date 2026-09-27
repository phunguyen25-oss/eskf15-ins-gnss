function tr = generate_truth(T,dt)
t=0:dt:T; n=numel(t);
tr.t=t;
tr.p=[4*sin(0.18*t); 3*cos(0.13*t)-3; -1.5-0.4*sin(0.11*t)];
tr.v=[0.72*cos(0.18*t); -0.39*sin(0.13*t); -0.044*cos(0.11*t)];
tr.a=[-0.1296*sin(0.18*t); -0.0507*cos(0.13*t); 0.00484*sin(0.11*t)];
roll=deg2rad(7)*sin(0.31*t); pitch=deg2rad(5)*sin(0.23*t+0.2);
yaw=deg2rad(20)*sin(0.09*t)+0.08*t;
rd=deg2rad(7)*0.31*cos(0.31*t);
pd=deg2rad(5)*0.23*cos(0.23*t+0.2);
yd=deg2rad(20)*0.09*cos(0.09*t)+0.08;
tr.q=zeros(4,n); tr.omega=zeros(3,n); tr.f=zeros(3,n);
g=[0;0;9.80665];
for k=1:n
    tr.q(:,k)=euler_zyx_to_quat(roll(k),pitch(k),yaw(k));
    tr.omega(:,k)=[1,0,-sin(pitch(k)); ...
        0,cos(roll(k)),sin(roll(k))*cos(pitch(k)); ...
        0,-sin(roll(k)),cos(roll(k))*cos(pitch(k))]*[rd(k);pd(k);yd(k)];
    tr.f(:,k)=quat_to_rot(tr.q(:,k))'*(tr.a(:,k)-g);
end
tr.ba=repmat([0.035;-0.022;0.018],1,n);
tr.bg=repmat(deg2rad([0.32;-0.18;0.25]),1,n);
end


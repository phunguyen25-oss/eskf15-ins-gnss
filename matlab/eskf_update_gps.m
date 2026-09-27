function [s,info] = eskf_update_gps(s,zp,zv,Rp,Rv,lever_b)
if nargin<6, lever_b=zeros(3,1); end
Rnb=quat_to_rot(s.q); lever_b=lever_b(:);
pred_p=s.p+Rnb*lever_b;
rotvel=cross(s.omega,lever_b);
pred_v=s.v+Rnb*rotvel;
Hp=zeros(3,15); Hp(:,1:3)=eye(3); Hp(:,7:9)=-Rnb*skew3(lever_b);
Hv=zeros(3,15); Hv(:,4:6)=eye(3);
Hv(:,7:9)=-Rnb*skew3(rotvel); Hv(:,13:15)=Rnb*skew3(lever_b);
[s,info]=eskf_update_sequential(s,[zp(:)-pred_p;zv(:)-pred_v], ...
    [Hp;Hv],[diag(Rp);diag(Rv)]);
end


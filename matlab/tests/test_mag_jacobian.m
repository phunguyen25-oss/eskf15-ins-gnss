function test_mag_jacobian()
q=euler_zyx_to_quat(0.3,-0.2,0.7); mn=[0.4;0.1;0.9]; mn=mn/norm(mn);
pred=quat_to_rot(q)'*mn; H=skew3(pred); eps0=1e-7; Hn=zeros(3);
for i=1:3
    d=zeros(3,1); d(i)=eps0;
    hp=quat_to_rot(quat_mul(q,quat_exp(d)))'*mn;
    hm=quat_to_rot(quat_mul(q,quat_exp(-d)))'*mn;
    Hn(:,i)=(hp-hm)/(2*eps0);
end
assert(norm(H-Hn,'fro')<1e-6);
end

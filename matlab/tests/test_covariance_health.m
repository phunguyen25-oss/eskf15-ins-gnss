function test_covariance_health()
c=eskf_default_config(); s=eskf_init(zeros(3,1),zeros(3,1),[1;0;0;0],zeros(3,1),zeros(3,1),eye(15),c);
for k=1:100
    s=eskf_predict(s,[0;0;-c.g],[0.01;0.02;-0.01],0.01);
    [s,~]=eskf_update_baro(s,0,0.2^2);
end
assert(norm(s.P-s.P','fro')<1e-10);
[~,p]=chol(s.P); assert(p==0);
end

